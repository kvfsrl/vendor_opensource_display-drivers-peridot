/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020 XiaoMi, Inc. All rights reserved.
 */

#define pr_fmt(fmt)	"mi-disp-lhbm:[%s:%d] " fmt, __func__, __LINE__
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <uapi/linux/sched/types.h>

#include "sde_trace.h"
#include "dsi_display.h"

#include "mi_disp_feature.h"
#include "mi_disp_print.h"
#include "mi_disp_lhbm.h"
#include "mi_dsi_display.h"
#include "mi_dsi_panel.h"
#include "mi_panel_id.h"
#include "sde_vm.h"
#include "mi_panel_id.h"


static struct disp_lhbm_fod *g_lhbm_fod[MI_DISP_MAX];

static int mi_disp_lhbm_fod_thread_fn(void *arg);
static int mi_disp_lhbm_fod_watch_thread_fn(void *arg);
static void hoshikv_fod_hold_expire_work(struct work_struct *work);
static void hoshikv_fod_hold_arm(struct disp_lhbm_fod *lhbm_fod);

bool is_local_hbm(int disp_id)
{
	struct dsi_display *display = NULL;

	if (is_support_disp_id(disp_id)) {
		if (disp_id == MI_DISP_PRIMARY)
			display = mi_get_primary_dsi_display();
		else
			display = mi_get_secondary_dsi_display();

		if (display && display->panel)
			return display->panel->mi_cfg.local_hbm_enabled;
		else
			return false;
	} else {
		DISP_ERROR("unknown display id\n");
		return false;
	}
}

bool mi_disp_lhbm_fod_enabled(struct dsi_panel *panel)
{
	return panel ? panel->mi_cfg.local_hbm_enabled : false;
}

/* forward decls: the fod_watch helpers below are static but used from
 * thread_create/thread_destroy which appear earlier in the file.
 */
static void hoshikv_fod_poll_qproc(struct file *fp,
		wait_queue_head_t *wq, struct poll_table_struct *pt);
static int hoshikv_fod_poll_wqfunc_entry(struct wait_queue_entry *entry,
		unsigned int mode, int flags, void *key);
static void hoshikv_fod_touch_close(struct disp_lhbm_fod *lhbm_fod);

int mi_disp_lhbm_fod_thread_create(struct disp_feature *df, int disp_id)
{
	int ret = 0;
	struct dsi_display *display = NULL;
	struct disp_lhbm_fod *lhbm_fod = NULL;

	if (!df || !is_support_disp_id(disp_id)) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	if (!df->d_display[disp_id].display ||
		df->d_display[disp_id].intf_type != MI_INTF_DSI) {
		DISP_ERROR("unsupported display(%s intf)\n",
			get_disp_intf_type_name(df->d_display[disp_id].intf_type));
		return -EINVAL;
	}

	display = (struct dsi_display *)df->d_display[disp_id].display;
	if (!mi_disp_lhbm_fod_enabled(display->panel)) {
		DISP_INFO("%s panel is not local hbm\n", get_disp_id_name(disp_id));
		return 0;
	}

	lhbm_fod = kzalloc(sizeof(struct disp_lhbm_fod), GFP_KERNEL);
	if (!lhbm_fod) {
		DISP_ERROR("can not allocate buffer for disp_lhbm\n");
		return -ENOMEM;
	}

	df->d_display[disp_id].lhbm_fod_ptr = lhbm_fod;
	lhbm_fod->display = display;

	INIT_LIST_HEAD(&lhbm_fod->event_list);
	spin_lock_init(&lhbm_fod->spinlock);
	INIT_DELAYED_WORK(&lhbm_fod->fod_hold_work,
			hoshikv_fod_hold_expire_work);

	atomic_set(&lhbm_fod->allow_tx_lhbm, 0);
	atomic_set(&lhbm_fod->target_brightness, LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);

	init_waitqueue_head(&lhbm_fod->fod_pending_wq);

	lhbm_fod->fod_thread = kthread_run(mi_disp_lhbm_fod_thread_fn,
			lhbm_fod, "disp_lhbm_fod:%d", disp_id);
	if (IS_ERR(lhbm_fod->fod_thread)) {
		DISP_ERROR("failed to create disp_fod:%d kthread\n", disp_id);
		ret = PTR_ERR(lhbm_fod->fod_thread);
		lhbm_fod->fod_thread = NULL;
		goto error;
	}
	/* set realtime priority */
	sched_set_fifo(lhbm_fod->fod_thread);

	/* hoshikv FOD watch */
	init_waitqueue_head(&lhbm_fod->fod_watch_wq);
	mutex_init(&lhbm_fod->fod_touch_lock);
	init_waitqueue_head(&lhbm_fod->fod_poll_wq);
	lhbm_fod->fod_poll_pt._qproc = hoshikv_fod_poll_qproc;
	lhbm_fod->fod_poll_pt._key = POLLPRI | POLLERR | POLLIN;
	init_waitqueue_func_entry(&lhbm_fod->fod_poll_entry,
			hoshikv_fod_poll_wqfunc_entry);
	lhbm_fod->fod_sysdev = df->d_display[disp_id].dev;
	atomic_set(&lhbm_fod->fod_watch_en, 0);
	atomic_set(&lhbm_fod->fod_press, 0);
	atomic_set(&lhbm_fod->fod_state_pub, 0);

	lhbm_fod->fod_watch_thread = kthread_run(
			mi_disp_lhbm_fod_watch_thread_fn, lhbm_fod,
			"disp_fod_watch:%d", disp_id);
	if (IS_ERR(lhbm_fod->fod_watch_thread)) {
		DISP_ERROR("failed to create disp_fod_watch:%d kthread\n",
			disp_id);
		ret = PTR_ERR(lhbm_fod->fod_watch_thread);
		lhbm_fod->fod_watch_thread = NULL;
	}

	g_lhbm_fod[disp_id] = lhbm_fod;

	DISP_INFO("create disp_lhbm_fod:%d kthread success\n", disp_id);

	return ret;

error:
	kfree(lhbm_fod);
	return ret;
}

int mi_disp_lhbm_fod_thread_destroy(struct disp_feature *df, int disp_id)
{
	int ret = 0;
	struct disp_lhbm_fod *lhbm_fod = NULL;

	if (!df || !is_support_disp_id(disp_id)) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	lhbm_fod = df->d_display[disp_id].lhbm_fod_ptr;
	if (lhbm_fod) {
		if (lhbm_fod->fod_watch_thread) {
			atomic_set(&lhbm_fod->fod_watch_en, 0);
			mutex_lock(&lhbm_fod->fod_touch_lock);
			hoshikv_fod_touch_close(lhbm_fod);
			mutex_unlock(&lhbm_fod->fod_touch_lock);
			wake_up_interruptible(&lhbm_fod->fod_watch_wq);
			wake_up_interruptible(&lhbm_fod->fod_poll_wq);
			kthread_stop(lhbm_fod->fod_watch_thread);
			lhbm_fod->fod_watch_thread = NULL;
		}
		if (lhbm_fod->fod_thread) {
			kthread_stop(lhbm_fod->fod_thread);
			lhbm_fod->fod_thread = NULL;
		}
		/* the hold worker dereferences lhbm_fod and its panel */
		cancel_delayed_work_sync(&lhbm_fod->fod_hold_work);
		kfree(lhbm_fod);
	}

	df->d_display[disp_id].lhbm_fod_ptr = NULL;
	g_lhbm_fod[disp_id] = NULL;

	DISP_INFO("destroy disp_lhbm_fod:%d kthread success\n", disp_id);

	return ret;
}

struct disp_lhbm_fod *mi_get_disp_lhbm_fod(int disp_id)
{
	if (is_support_disp_id(disp_id)) {
		return g_lhbm_fod[disp_id];
	} else {
		DISP_ERROR("unknown display id\n");
		return NULL;
	}
}

int mi_disp_lhbm_fod_allow_tx_lhbm(struct dsi_display *display,
		bool enable)
{
	struct disp_lhbm_fod *lhbm_fod = NULL;

	if (!display) {
		DISP_ERROR("Invalid display ptr\n");
		return -EINVAL;
	}

	if (!mi_disp_lhbm_fod_enabled(display->panel)) {
		DISP_DEBUG("%s panel is not local hbm\n", display->display_type);
		return 0;
	}

	lhbm_fod = mi_get_disp_lhbm_fod(mi_get_disp_id(display->display_type));
	if (!lhbm_fod) {
		DISP_ERROR("Invalid lhbm_fod ptr\n");
		return -EINVAL;
	}

	if (lhbm_fod->display == display &&
		atomic_read(&lhbm_fod->allow_tx_lhbm) != enable) {
		atomic_set(&lhbm_fod->allow_tx_lhbm, enable);
		DISP_INFO("%s display allow_tx_lhbm = %d\n", display->display_type, enable);
		if (enable) {
			wake_up_interruptible(&lhbm_fod->fod_pending_wq);
			/* if target_brightness is saved, will restor the local hbm on */
			if (atomic_read(&lhbm_fod->disp_off_target_brightness) != LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
				atomic_read(&lhbm_fod->disp_off_target_brightness) != LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
				mi_disp_set_local_hbm(mi_get_disp_id(display->display_type),
						atomic_read(&lhbm_fod->disp_off_target_brightness));
			}
			DISP_INFO("%s display wake up local disp_fod kthread\n", display->display_type);
		} else {
			/* If the last fod fingerprint status is that the finger is pressed and
                         * the finger is not lifted when display off, save target_brightness */
			if (atomic_read(&lhbm_fod->target_brightness) != LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
				atomic_read(&lhbm_fod->target_brightness) != LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP &&
				list_empty(&lhbm_fod->event_list)) {
				atomic_set(&lhbm_fod->disp_off_target_brightness, atomic_read(&lhbm_fod->target_brightness));
			} else {
				atomic_set(&lhbm_fod->disp_off_target_brightness, LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
			}
			atomic_set(&lhbm_fod->target_brightness, LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
		}
	}
	return 0;
}

int mi_disp_lhbm_fod_update_layer_state(struct dsi_display *display,
		struct mi_layer_flags flags)
{
	struct disp_lhbm_fod *lhbm_fod = NULL;

	if (!display) {
		DISP_ERROR("Invalid display ptr\n");
		return -EINVAL;
	}

	if (!mi_disp_lhbm_fod_enabled(display->panel)) {
		DISP_DEBUG("%s panel is not local hbm\n", display->display_type);
		return 0;
	}

	lhbm_fod = mi_get_disp_lhbm_fod(mi_get_disp_id(display->display_type));
	if (!lhbm_fod) {
		DISP_ERROR("Invalid lhbm_fod ptr\n");
		return -EINVAL;
	}

	spin_lock(&lhbm_fod->spinlock);
	lhbm_fod->layer_flags = flags;
	spin_unlock(&lhbm_fod->spinlock);

	/* hoshikv doze2: no AOD surface left -> stop holding doze brightness */
	if (!flags.aod_present && !flags.gxzw_anim_present)
		mi_dsi_hoshikv_doze_drop(display);

	return 0;
}

static int mi_disp_lhbm_fod_event_notify(struct disp_lhbm_fod *lhbm_fod, int fod_status)
{
	struct dsi_display *display = NULL;
	int disp_id = MI_DISP_PRIMARY;
	u32 fod_ui_ready = 0, refresh_rate = 0;
	u32 ui_ready_delay_frame = 0;
	u64 delay_us = 0;

	if (!lhbm_fod || !lhbm_fod->display) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	display = lhbm_fod->display;

	if (!display->panel || !display->panel->cur_mode){
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}
	refresh_rate = display->panel->cur_mode->timing.refresh_rate;
	/*
	 * hoshikv: was `refresh_rate == 30`. That only covered the AOD rate, so a
	 * panel sitting at 60Hz fell through and local HBM was injected at 60Hz,
	 * which greys the screen until finger-up. Ask the HAL for the FOD rate
	 * whenever we are not already at it.
	 */
	if (fod_status == FOD_EVENT_FPS &&
		refresh_rate < NEED_UPDATE_TO_FOD_FPS) {
		fod_ui_ready = LOCAL_HBM_NEED_UPDATE_TO_FOD_FPS;
		mi_disp_feature_event_notify_by_type(disp_id, MI_DISP_EVENT_FOD,
				sizeof(fod_ui_ready), fod_ui_ready);
		return -NEED_UPDATE_TO_FOD_FPS;
	}

	if (fod_status == FOD_EVENT_DOWN &&
		display->panel->mi_cfg.lhbm_ui_ready_delay_frame > 0) {
		ui_ready_delay_frame = display->panel->mi_cfg.lhbm_ui_ready_delay_frame;
		delay_us = 1000000 / refresh_rate * ui_ready_delay_frame;
		DISP_INFO("refresh_rate(%d), delay (%d) frame, delay_us(%llu)\n",
				refresh_rate, ui_ready_delay_frame, delay_us);
		usleep_range(delay_us, delay_us + 10);
	}

	if (fod_status == FOD_EVENT_DOWN) {
		if (atomic_read(&lhbm_fod->target_brightness) == LHBM_TARGET_BRIGHTNESS_WHITE_110NIT)
			fod_ui_ready = LOCAL_HBM_UI_READY | FOD_LOW_BRIGHTNESS_CAPTURE;
		else
			fod_ui_ready = LOCAL_HBM_UI_READY;
	} else {
		fod_ui_ready = LOCAL_HBM_UI_NONE;
	}

	if (atomic_read(&lhbm_fod->allow_tx_lhbm)) {
		disp_id = mi_get_disp_id(display->display_type);
		mi_disp_feature_event_notify_by_type(disp_id, MI_DISP_EVENT_FOD,
				sizeof(fod_ui_ready), fod_ui_ready);

		DISP_INFO("%s display fod_ui_ready notify=%d\n",
			display->display_type, fod_ui_ready);
	}

	return 0;
}

static bool hoshikv_fod_in_aod(struct dsi_panel *panel,
		struct disp_lhbm_fod *lhbm_fod);

static int mi_disp_lhbm_fod_set_disp_param(struct disp_lhbm_fod *lhbm_fod, u32 lhbm_value)
{
	struct dsi_panel *panel = NULL;
	struct mi_dsi_panel_cfg *mi_cfg = NULL;
	struct disp_feature_ctl ctl;
	struct dsi_display *dsi_display = NULL;
	struct sde_kms *sde_kms = NULL;
	int rc = 0;

	if (!lhbm_fod || !lhbm_fod->display || !lhbm_fod->display->panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}
	dsi_display = lhbm_fod->display;
	memset(&ctl, 0, sizeof(struct disp_feature_ctl));
	panel = lhbm_fod->display->panel;

	if (sde_kms_is_suspend_blocked(dsi_display->drm_dev)) {
		DISP_ERROR("sde_kms is suspended, skip to set disp_param\n");
		return -EBUSY;
	}

	sde_kms = dsi_display_get_kms(dsi_display);
	if (sde_kms) {
		sde_vm_lock(sde_kms);
		if (!sde_vm_owns_hw(sde_kms)) {
			DISP_ERROR("op not supported due to HW unavailablity\n");
			rc = -EOPNOTSUPP;
			goto end;
		}
	}

	mutex_lock(&panel->panel_lock);

	mi_cfg = &panel->mi_cfg;

	switch (lhbm_value) {
	case LHBM_TARGET_BRIGHTNESS_WHITE_1000NIT:
	case LHBM_TARGET_BRIGHTNESS_WHITE_110NIT:
	case LHBM_TARGET_BRIGHTNESS_GREEN_500NIT:
		ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
		if (lhbm_value == LHBM_TARGET_BRIGHTNESS_GREEN_500NIT) {
			ctl.feature_val = LOCAL_HBM_NORMAL_GREEN_500NIT;
		} else if (lhbm_value == LHBM_TARGET_BRIGHTNESS_WHITE_1000NIT) {
			if (hoshikv_fod_in_aod(panel, lhbm_fod) &&
				(mi_cfg->panel_state == PANEL_STATE_DOZE_HIGH
				||mi_cfg->panel_state == PANEL_STATE_DOZE_LOW
				||lhbm_fod->fod_nolp_on
				||lhbm_fod->fod_sdm_doze))
				ctl.feature_val = LOCAL_HBM_HLPM_WHITE_1000NIT;
			else
				ctl.feature_val = LOCAL_HBM_NORMAL_WHITE_1000NIT;
		} else if (lhbm_value == LHBM_TARGET_BRIGHTNESS_WHITE_110NIT) {
			if (hoshikv_fod_in_aod(panel, lhbm_fod) &&
				(mi_cfg->panel_state == PANEL_STATE_DOZE_HIGH
				||mi_cfg->panel_state == PANEL_STATE_DOZE_LOW
				||lhbm_fod->fod_nolp_on
				||lhbm_fod->fod_sdm_doze))
				ctl.feature_val = LOCAL_HBM_HLPM_WHITE_110NIT;
			else
				ctl.feature_val = LOCAL_HBM_NORMAL_WHITE_110NIT;
		} else {
			DISP_ERROR("invalid target_brightness = %d\n", lhbm_fod->target_brightness);
		}
		break;
	case LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP:
		ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
		if (hoshikv_fod_in_aod(panel, lhbm_fod))
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL_BACKLIGHT_RESTORE;
		else
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL;
		break;
	case LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP:
		ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
		if (hoshikv_fod_in_aod(panel, lhbm_fod)) {
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL_BACKLIGHT;
		} else {
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL;
		}
		break;
	default:
		break;
	}

	atomic_set(&lhbm_fod->target_brightness, lhbm_value);
	rc = mi_dsi_panel_set_lhbm_fod_locked(panel, &ctl);
	mi_cfg->feature_val[DISP_FEATURE_LOCAL_HBM] = ctl.feature_val;
	mutex_unlock(&panel->panel_lock);

end:
	if (sde_kms)
		sde_vm_unlock(sde_kms);

	return rc;
}

int mi_disp_lhbm_aod_to_normal_optimize(struct dsi_display *display,
		bool enable)
{
	struct disp_feature_ctl ctl;
	int rc = 0;

	if (!display || !display->panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	if (!display->panel->mi_cfg.need_fod_animal_in_normal)
		return 0;

	memset(&ctl, 0, sizeof(struct disp_feature_ctl));
	ctl.feature_id = DISP_FEATURE_AOD_TO_NORMAL;
	ctl.feature_val = enable ? FEATURE_ON : FEATURE_OFF;

	rc = mi_dsi_display_set_disp_param(display, &ctl);

	return rc;
}

static bool mi_disp_lhbm_fod_thread_should_wake(struct disp_lhbm_fod *lhbm_fod)
{
	bool should_wake = false;
	unsigned long flags;

	spin_lock_irqsave(&lhbm_fod->spinlock, flags);

	if (list_empty(&lhbm_fod->event_list) || !atomic_read(&lhbm_fod->allow_tx_lhbm))
		should_wake = false;
	else
		should_wake = true;

	spin_unlock_irqrestore(&lhbm_fod->spinlock, flags);

	return should_wake;
}

static int mi_disp_lhbm_fod_thread_fn(void *arg)
{
	int rc = 0;
	struct disp_lhbm_fod *lhbm_fod = (struct disp_lhbm_fod *)arg;
	struct lhbm_setting *entry = NULL, *temp = NULL;
	struct lhbm_setting lhbm_setting_event;
	unsigned long flag = 0;

	while (!kthread_should_stop()) {
		rc = wait_event_interruptible(lhbm_fod->fod_pending_wq,
				mi_disp_lhbm_fod_thread_should_wake(lhbm_fod));
		if (rc) {
			/* Some event woke us up */
			DISP_WARN("wait_event_interruptible rc = %d\n", rc);
			continue;
		}

		spin_lock_irqsave(&lhbm_fod->spinlock, flag);
		entry = list_last_entry(&lhbm_fod->event_list, struct lhbm_setting, link);
		DISP_INFO("lhbm_value(%d)\n", entry->lhbm_value);
		memcpy(&lhbm_setting_event, entry, sizeof(lhbm_setting_event));
		/* hoshikv: the Xiaomi FOD_EVENT_FPS handshake is gone. It used to
		 * ask the HAL to raise doze 30->120 before a press, and on a missing
		 * handshake it killed allow_tx_lhbm so the queued DOWN/HBM was never
		 * applied. The DRM clock never moves for the AOD doze mode, so that
		 * gate always wedged here. doze 120Hz is now driven by our own
		 * DOZE_HBM command before the HBM is queued, so the event below is
		 * always processed. */
		list_for_each_entry_safe(entry, temp, &lhbm_fod->event_list, link) {
			DISP_DEBUG("in list, lhbm_value(%d)\n", entry->lhbm_value);
			list_del(&entry->link);
			kfree(entry);
		}
		if (atomic_read(&lhbm_fod->target_brightness) != lhbm_setting_event.lhbm_value) {
			atomic_set(&lhbm_fod->target_brightness, lhbm_setting_event.lhbm_value);

			spin_unlock_irqrestore(&lhbm_fod->spinlock, flag);

			if (lhbm_setting_event.lhbm_value == LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP ||
				lhbm_setting_event.lhbm_value == LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
				mi_disp_lhbm_fod_event_notify(lhbm_fod, FOD_EVENT_UP);
			}

			rc = mi_disp_lhbm_fod_set_disp_param(lhbm_fod, lhbm_setting_event.lhbm_value);
			if (rc) {
				DISP_ERROR("lhbm_fod failed to set_disp_param, rc = %d\n", rc);
			} else if (lhbm_setting_event.lhbm_value != LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
				lhbm_setting_event.lhbm_value != LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
				mi_disp_lhbm_fod_event_notify(lhbm_fod, FOD_EVENT_DOWN);
			}
		} else {
			spin_unlock_irqrestore(&lhbm_fod->spinlock, flag);
			DISP_INFO("same lhbm setting event: %d, return\n", lhbm_setting_event.lhbm_value);
		}
	}

	return 0;
}

int mi_disp_set_local_hbm(int disp_id, int lhbm_value)
{
	struct disp_lhbm_fod *lhbm_fod = mi_get_disp_lhbm_fod(disp_id);
	struct lhbm_setting *lhbm_setting_event = NULL, *entry = NULL;
	unsigned long flags;
	int rc = 0;

#ifdef CONFIG_FACTORY_BUILD
	return 0;
#endif

	if (!is_local_hbm(disp_id)) {
		DISP_DEBUG("%s panel is not local hbm\n", get_disp_id_name(disp_id));
		return 0;
	}

	if (!lhbm_fod) {
		DISP_ERROR("invalid lhbm_fod ptr\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&lhbm_fod->spinlock, flags);

	mi_disp_feature_event_notify_by_type(disp_id,
			MI_DISP_EVENT_LOCAL_HBM_VALUE, sizeof(lhbm_value), lhbm_value);

	if (lhbm_value == LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP ||
			lhbm_value == LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
		atomic_set(&lhbm_fod->disp_off_target_brightness, lhbm_value);
	}

	if (atomic_read(&lhbm_fod->target_brightness) == LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
			lhbm_value == LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
		DISP_INFO("skip set LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP\n");
		goto exit;
	}

	lhbm_setting_event = kzalloc(sizeof(struct lhbm_setting), GFP_ATOMIC);
	if (!lhbm_setting_event) {
		DISP_ERROR("failed to allocate memory for lhbm_setting_event\n");
		rc = ENOMEM;
		goto exit;
	}

	lhbm_setting_event->lhbm_value = lhbm_value;
	INIT_LIST_HEAD(&lhbm_setting_event->link);
	list_add_tail(&lhbm_setting_event->link, &lhbm_fod->event_list);

	list_for_each_entry(entry, &lhbm_fod->event_list, link) {
		DISP_DEBUG("in list, lhbm_value(%d)\n", entry->lhbm_value);
	}

	DISP_INFO("local_hbm_value:%s\n", get_lhbm_value_name(lhbm_value));
	wake_up_interruptible(&lhbm_fod->fod_pending_wq);

exit:
	spin_unlock_irqrestore(&lhbm_fod->spinlock, flags);
	return rc;
}

int mi_disp_update_0size_lhbm_info(struct dsi_panel *panel)
{
	struct mi_dsi_panel_cfg *mi_cfg  = NULL;
	struct disp_feature_ctl ctl;
	int rc = 0;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	if (mi_get_panel_id_by_dsi_panel(panel) != N3_PANEL_PA)
		return rc;

	mi_cfg = &panel->mi_cfg;

	if (is_hbm_fod_on(panel))
	{
		DISP_DEBUG("skip 0 size lhbm due to lhbm is on\n");
		return rc;
	}
	if (panel->power_mode == SDE_MODE_DPMS_ON) {
		memset(&ctl, 0, sizeof(struct disp_feature_ctl));
		if(mi_cfg->lhbm_gxzw && !mi_cfg->lhbm_0size_on && mi_cfg->feature_val[DISP_FEATURE_FP_STATUS] != AUTH_STOP) {
			if (mi_cfg->last_bl_level) {
				rc = mi_dsi_panel_set_lhbm_0size_locked(panel);
			}
			mi_cfg->lhbm_0size_on = true;
			DISP_DEBUG("gxzw apper,0 size lhbm on,last_bl_level[%d]\n",mi_cfg->last_bl_level);
		} else if (!mi_cfg->lhbm_gxzw && mi_cfg->lhbm_0size_on) {
			ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL_BACKLIGHT;
			rc = mi_dsi_panel_set_lhbm_fod_locked(panel, &ctl);
			panel->mi_cfg.feature_val[DISP_FEATURE_LOCAL_HBM] = ctl.feature_val;
			mi_cfg->lhbm_0size_on = false;
			DISP_DEBUG("gxzw quit,0 size lhbm off to normal,last_bl_level[%d]\n",mi_cfg->last_bl_level);
		}
	}

	return rc;
}

int mi_disp_update_0size_lhbm_layer(struct dsi_display *dsi_display,
			u32 mi_gxzw_flags)
{
	struct dsi_panel *panel = NULL;
	struct mi_dsi_panel_cfg *mi_cfg  = NULL;
	int rc = 0;

	if (!dsi_display || !dsi_display->panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	panel = dsi_display->panel;

	if (mi_get_panel_id_by_dsi_panel(panel) != N3_PANEL_PA)
		return rc;

	mi_cfg = &panel->mi_cfg;
	dsi_panel_acquire_panel_lock(panel);
	rc = mi_disp_update_0size_lhbm_info(panel);
	dsi_panel_release_panel_lock(panel);

	return rc;
}


/* ===================== hoshikv FOD-HBM watch ===================== */

/*
 * Notification-driven FOD capture, wired to libhoshikv:
 *   - the fod_watch kthread is armed/stopped ONLY by
 *     MI_DISP_IOCTL_SET_FOD_MODE (disp_feature_req.feature_val), mirroring
 *     libhoshikv k()/v(). No auto-arm at AOD entry and no polling of
 *     /dev/xiaomi-touch mode 10.
 *   - SET_FOD_MODE on == forced press: HBM on + state=1, but ONLY while
 *     the screen is fully awake (the touch driver drops FOD events awake).
 *     In doze/OFF the arm never lights HBM; the touch watcher below is the
 *     sole trigger there, reacting to real fod_press_status edges only.
 *   - SET_FOD_MODE off == release: HBM off and state=0.
 *   - while armed the kthread is the SOLE reader of fod_press_status. Instead
 *     of polling it injects a wait_queue_entry into the node's kernfs poll
 *     waitqueue (via the file's ->poll()) so it wakes on each sysfs_notify.
 *     touch driver calls notify_oneshot_sensor(FOD_PRESS, 1) on press and
 *     (..., 0) on release, both followed by sysfs_notify, so each edge is
 *     observed exactly once; the oneshot value is then consumed by a single
 *     read. No touch driver patch, no symbol_request.
 *   - press  -> mirror hoshikv_fod_state to 1 + sysfs_notify (lib
 *               poll(POLLPRI) -> onFpTouch(true)), doze 120Hz + local HBM
 *   - release -> mirror 0 + sysfs_notify (onFpTouch(false)), HBM off,
 *               hold doze 120Hz until HOSHIKV_FOD_HOLD_MS then 30Hz
 *   - the watcher only acts on value transitions and always sleeps between
 *     reads, so nodes that keep poll() ready forever cannot busy-loop it.
 */

/*
 * Protection: only inject HBM once the panel is actually parked in doze.
 * power_mode DPMS_OFF (5) means the panel is mid-transition and any HBM
 * injection there greys the screen.
 */
static bool hoshikv_fod_panel_stable(struct dsi_panel *panel)
{
	if (!panel) {
		DISP_INFO("hoshikv-fod: no panel, skip HBM inj\n");
		return false;
	}

	if (!dsi_panel_initialized(panel)) {
		DISP_INFO("hoshikv-fod: panel not initialized, skip HBM inj\n");
		return false;
	}

	if (panel->power_mode != SDE_MODE_DPMS_ON &&
		panel->power_mode != SDE_MODE_DPMS_LP1 &&
		panel->power_mode != SDE_MODE_DPMS_LP2) {
		DISP_INFO("hoshikv-fod: panel power_mode=%d not stable,"
			" skip HBM inj\n", panel->power_mode);
		return false;
	}

	return true;
}

/*
 * AOD context for the LHBM feature values. Normally that is
 * is_aod_and_panel_initialized() (LP1/LP2 + panel_state DOZE_*). While the FOD
 * press owns doze NOLP the panel is out of LP with power_mode faked to
 * DPMS_ON, but it still renders AOD/FOD content, so the AOD (HLPM) command
 * variants must still be used or the HBM off would restore normal backlight.
 */
static bool hoshikv_fod_in_aod(struct dsi_panel *panel,
		struct disp_lhbm_fod *lhbm_fod)
{
	if (is_aod_and_panel_initialized(panel))
		return true;

	return lhbm_fod && lhbm_fod->fod_nolp_on &&
		dsi_panel_initialized(panel);
}

/*
 * doze NOLP via the stock Xiaomi path: mi_dsi_panel_aod_to_normal_optimize_
 * locked() is the panel's own aod->normal switch. It sends the DOZE_HBM_NOLP
 * command set (5f 40 + 51 <doze hbm dbv>) so the panel leaves LP and scans out
 * at full rate, and it keeps the vendor bookkeeping (panel_state = ON,
 * aod_to_normal_statue) consistent so dsi_panel_set_lp2() and
 * mi_dsi_panel_set_doze_brightness() still behave. Unlike
 * dsi_panel_set_nolp() it does not fake power_mode = DPMS_ON, so the matching
 * disable path stays legal.
 *
 * Stock only reaches this from sde_encoder on a VRR non-30 atomic commit,
 * and nothing on this device commits one while the panel is in BLANK_LP, so
 * we drive the same function directly from the touch path.
 */
/*
 * Full AOD means SDM itself parks the display in doze and drives the panel
 * rate/gamma. Any driver-side doze NOLP on top of that fights SDM: we would
 * send DOZE_HBM, leave LP, then the next SDM doze commit yanks the panel back
 * to 30Hz aod gamma while FOD HBM is still lit -> that is the green flash.
 * So while an AOD surface is up, the driver owns nothing and reports so.
 */
static bool hoshikv_fod_in_doze(struct disp_lhbm_fod *lhbm_fod)
{
	struct dsi_panel *panel = lhbm_fod->display->panel;

	/*
	 * Any doze, whatever the AOD flavour is (static, seamless, screen-off
	 * aod, or the fingerprint ui). Do not use layer_flags here, it is never
	 * populated on this panel id. LP1/LP2 + initialized is the only reliable
	 * signal. This is the "may I ask SDM for the transition" predicate, used
	 * on press.
	 */
	return is_aod_and_panel_initialized(panel);
}

bool mi_disp_lhbm_fod_sdm_doze_active(struct dsi_display *display)
{
	struct disp_lhbm_fod *lhbm_fod;
	struct dsi_panel *panel;

	if (!display)
		return false;

	panel = display->panel;
	if (!panel)
		return false;

	if (panel->power_mode != SDE_MODE_DPMS_LP1 &&
	    panel->power_mode != SDE_MODE_DPMS_LP2)
		return false; /* not doze: SDM is not driving anything */

	lhbm_fod = mi_get_disp_lhbm_fod(mi_get_disp_id(display->display_type));
	if (!lhbm_fod)
		return false;

	/*
	 * Only while a FOD press actually owns doze. A plain screen-off doze is
	 * NOT this: the driver still has to arm DOZE_HBM/DBV there, or the
	 * panel comes up with no doze gamma at all and AOD only shows up after
	 * the first FOD press has forced it.
	 */
	return hoshikv_fod_in_doze(lhbm_fod) && lhbm_fod->fod_sdm_doze;
}

static int hoshikv_fod_doze_nolp_enter(struct disp_lhbm_fod *lhbm_fod)
{
	struct dsi_panel *panel = lhbm_fod->display->panel;
	int rc;

	if (lhbm_fod->fod_nolp_on)
		return 0;

	if (panel->power_mode == SDE_MODE_DPMS_ON)
		return 0; /* awake: nothing to borrow, HAL owns the rate */

	/*
	 * Every doze flavour is SDM's here, so do not send DOZE_HBM /
	 * DOZE_HBM_NOLP from the driver at all. Ask SDM for the transition via
	 * the feature path and let the 3s hold keep it. Going straight to the
	 * panel while SDM is mid-ramp is what tore the panel out of the aod
	 * gamma under the lit fingerprint and flashed green.
	 */
	if (hoshikv_fod_in_doze(lhbm_fod)) {
		struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;

		/*
		 * mi_dsi_panel_aod_to_normal_optimize_locked() picks the panel
		 * command off mi_cfg->doze_brightness. If that is still LBM we
		 * would ask SDM for DOZE_LBM_NOLP, and forcing the LBM doze mode
		 * to NOLP on top of the aod gamma is the full green screen. Pin
		 * HBM first so SDM is guaranteed to send DOZE_HBM_NOLP.
		 */
		if (mi_cfg->doze_brightness != DOZE_BRIGHTNESS_HBM) {
			mi_cfg->last_doze_brightness = mi_cfg->doze_brightness;
			mi_cfg->doze_brightness = DOZE_BRIGHTNESS_HBM;
		}

		/*
		 * hoshikv: force a fresh DOZE_HBM_NOLP send on every press.
		 * panel_state can still read PANEL_STATE_ON from a stale
		 * aod_to_normal_statue left over from an earlier cycle: when SDM
		 * drives the panel back to the doze gamma directly, the cmd-set
		 * path (dsi_panel_set_lp2) skips because the flag says the panel is
		 * already in aod, so panel_state/aod_to_normal_statue are never
		 * reset to a doze value. On the next press the feature handler
		 * then hits "enable while already ON" -> -EAGAIN and never sends
		 * DOZE_HBM_NOLP, so the HBM lights on a panel that is still in the
		 * lp aod gamma -> greying/green tint. Reset the flag to the doze
		 * state first so the handler takes its real enable branch and
		 * re-sends the command set (it sets panel_state back to ON on
		 * success).
		 */
		mi_cfg->panel_state = PANEL_STATE_DOZE_HIGH;
		mi_cfg->aod_to_normal_statue = false;

		rc = mi_disp_lhbm_aod_to_normal_optimize(lhbm_fod->display,
				true);

		/*
		 * hoshikv: -EAGAIN from the feature handler is NOT a failure here.
		 * mi_dsi_panel_aod_to_normal_optimize_locked() falls into its final
		 * else (mi_dsi_panel.c:4206) when it is asked to enable while the
		 * panel is already PANEL_STATE_ON, i.e. it declined because the
		 * DOZE_HBM_NOLP set is already in place -- precisely the state we
		 * want.
		 *
		 * Judging the outcome on rc left aod_to_normal_statue false while the
		 * panel was in fact on. dsi_panel_set_lp1() consults that flag before
		 * dragging the panel back into lp, so it pulled the panel straight
		 * back to 30Hz and doze 120Hz never lasted through the touch.
		 * panel_state is the ground truth: normalise rc from it.
		 */
		if (mi_cfg->panel_state == PANEL_STATE_ON) {
			rc = 0;
		} else {
			int tries;

			/*
			 * Retry here instead of leaving it to sde_encoder.c: that
			 * retry only runs on a committed video frame, and a FOD press
			 * out of doze commits no frame. So a request that never moved
			 * the panel out of doze would just sit pending forever: HBM
			 * lit at 1000nit on a panel still sitting in LP2 aod gamma,
			 * blinking grey/green on every press, never recovering.
			 */
			for (tries = HOSHIKV_FOD_FPS_TRIES; tries > 0; tries--) {
				usleep_range(8000, 12000);
				rc = mi_disp_lhbm_aod_to_normal_optimize(
						lhbm_fod->display, true);
				if (mi_cfg->panel_state == PANEL_STATE_ON) {
					rc = 0;
					break;
				}
			}
			if (mi_cfg->panel_state != PANEL_STATE_ON)
				DISP_ERROR("hoshikv-fod: SDM doze NOLP failed"
					" after %d tries (rc=%d, state=%d)\n",
					HOSHIKV_FOD_FPS_TRIES - tries, rc,
					mi_cfg->panel_state);
		}

		/*
		 * aod_to_normal_statue is what dsi_panel_set_lp1() checks
		 * before dragging the panel back into lp (dsi_panel.c:5130).
		 * Derive it from panel_state alone, never from rc, or a panel that
		 * is genuinely out of doze still gets pulled back into lp.
		 */
		mi_cfg->aod_to_normal_statue =
			(mi_cfg->panel_state == PANEL_STATE_ON);
		mi_cfg->aod_to_normal_pending = !mi_cfg->aod_to_normal_statue;

		DISP_INFO("hoshikv-fod: SDM doze requested (rc=%d, mode=%d,"
			" state=%d, pending=%d)\n", rc, panel->power_mode,
			mi_cfg->panel_state, mi_cfg->aod_to_normal_pending);

		if (!mi_cfg->aod_to_normal_statue) {
			/* hoshikv: never claim ownership we do not have. Returning
			 * the real failure lets the caller retry and keeps the HBM
			 * from being queued on a panel still sitting in doze. */
			return -EAGAIN;
		}

		/*
		 * hoshikv: only claim SDM doze ownership once the panel really left
		 * doze. Setting fod_sdm_doze before knowing the outcome is what made
		 * the release path run (and drop the panel back to 30Hz) 432us after
		 * the press, before the 3s hold could ever be observed.
		 */
		lhbm_fod->fod_sdm_doze = 1;

		return 0;
	}

	if (!is_aod_and_panel_initialized(panel)) {
		/* in lp but the doze command sets have not landed yet. Returning 0
		 * here is what let the caller inject HBM while the panel was still
		 * in the aod gamma, which is the "hbm + grey together" case. */
		return -EAGAIN;
	}

	if (!panel->mi_cfg.need_fod_animal_in_normal) {
		DISP_ERROR("hoshikv-fod: vendor aod->normal disabled in dt\n");
		return -ENOTSUPP;
	}

	/* DOZE_HBM_NOLP is the HBM variant of the command set */
	if (panel->mi_cfg.doze_brightness != DOZE_BRIGHTNESS_HBM) {
		panel->mi_cfg.last_doze_brightness =
			panel->mi_cfg.doze_brightness;
		panel->mi_cfg.doze_brightness = DOZE_BRIGHTNESS_HBM;
	}

	lhbm_fod->fod_nolp_prev_mode = panel->power_mode;
	/* claim the doze ui BEFORE dropping into the vendor switch: a doze commit
	 * landing after the aod exit but before we set the flag would push the
	 * panel straight back into the aod gamma underneath the hbm. */
	lhbm_fod->fod_nolp_on = 1;

	mutex_lock(&panel->panel_lock);
	/* leaves the aod, then the gamma, then the colour mode, then nolp, as one
	 * pipelined burst so the panel is never sitting outside aod without nolp */
	rc = mi_dsi_panel_fod_nolp_enter_locked(panel);
	mutex_unlock(&panel->panel_lock);

	/* the function no-ops when the fod hbm is already on, so confirm the
	 * panel really left lp before claiming we did */
	if (rc || !panel->mi_cfg.aod_to_normal_statue) {
		DISP_ERROR("hoshikv-fod: doze NOLP refused (rc=%d,"
			" hbm_fod=%d)\n", rc, is_hbm_fod_on(panel));
		lhbm_fod->fod_nolp_prev_mode = 0;
		lhbm_fod->fod_nolp_on = 0;
		return rc ? rc : -EAGAIN;
	}

	DISP_INFO("hoshikv-fod: doze NOLP on (mode=%d)\n",
		lhbm_fod->fod_nolp_prev_mode);

	return 0;
}

/* back to LP doze 30Hz after the hold expired. */
static void hoshikv_fod_doze_nolp_leave(struct disp_lhbm_fod *lhbm_fod)
{
	struct dsi_panel *panel = lhbm_fod->display->panel;
	int prev_mode;
	int rc;
	int aod_rc;
	int tries;

	/*
	 * Hold expired. Ask SDM to put the panel back into doze (30Hz); do not
	 * drive the panel from here or we land the same aod->normal gamma the
	 * fingerprint was lit against, which is the green flash.
	 */
	if (lhbm_fod->fod_sdm_doze) {
		struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;
		int sdm_rc;

		lhbm_fod->fod_nolp_on = 0;
		lhbm_fod->fod_nolp_prev_mode = 0;
		lhbm_fod->fod_sdm_doze = 0;

		/*
		 * Hand the depth back the way we found it. The handler switches
		 * on doze_brightness to pick DOZE_HBM, so keep HBM pinned for the
		 * release too, otherwise it would drop to the default branch and
		 * never send anything.
		 */
		if (mi_cfg->doze_brightness != DOZE_BRIGHTNESS_HBM)
			mi_cfg->doze_brightness = DOZE_BRIGHTNESS_HBM;

		sdm_rc = mi_disp_lhbm_aod_to_normal_optimize(lhbm_fod->display,
				false);

		/* same reason as the acquire path: no frame is going to commit
		 * here, so a pending release would never be retried and the
		 * panel would stay stuck out of doze with the fingerprint ui
		 * painted over it. Retry in-thread, bounded. */
		if (sdm_rc != 0 ||
			mi_cfg->panel_state != PANEL_STATE_DOZE_HIGH) {
			int tries;

			for (tries = HOSHIKV_FOD_FPS_TRIES; tries > 0; tries--) {
				usleep_range(8000, 12000);
				sdm_rc = mi_disp_lhbm_aod_to_normal_optimize(
						lhbm_fod->display, false);
				if (!sdm_rc &&
					mi_cfg->panel_state == PANEL_STATE_DOZE_HIGH)
					break;
			}
			if (sdm_rc ||
				mi_cfg->panel_state != PANEL_STATE_DOZE_HIGH)
				DISP_ERROR("hoshikv-fod: SDM doze release failed"
					" after %d tries (rc=%d, state=%d)\n",
					HOSHIKV_FOD_FPS_TRIES - tries, sdm_rc,
					mi_cfg->panel_state);
		}

		mi_cfg->aod_to_normal_statue =
			(!sdm_rc &&
			 mi_cfg->panel_state == PANEL_STATE_DOZE_HIGH);
		mi_cfg->aod_to_normal_pending = !mi_cfg->aod_to_normal_statue;
		DISP_INFO("hoshikv-fod: SDM doze released (rc=%d, mode=%d,"
				" state=%d, pending=%d)\n", sdm_rc,
				panel->power_mode, mi_cfg->panel_state,
				mi_cfg->aod_to_normal_pending);
		return;
	}

	if (!lhbm_fod->fod_nolp_on)
		return;

	lhbm_fod->fod_nolp_on = 0;
	prev_mode = lhbm_fod->fod_nolp_prev_mode;
	lhbm_fod->fod_nolp_prev_mode = 0;

	if (!dsi_panel_initialized(panel) ||
	    (panel->power_mode != SDE_MODE_DPMS_LP1 &&
	     panel->power_mode != SDE_MODE_DPMS_LP2)) {
		/* the display already took the panel over (real screen on/off),
		 * only drop our bookkeeping */
		DISP_INFO("hoshikv-fod: doze NOLP off (dpms took over,"
			" mode=%d)\n", panel->power_mode);
		return;
	}

	/* restore the idle doze level first: the disable branch picks its command
	 * set from doze_brightness, so this lands as DOZE_LBM again */
	if (panel->mi_cfg.last_doze_brightness != DOZE_TO_NORMAL)
		panel->mi_cfg.doze_brightness =
			panel->mi_cfg.last_doze_brightness;

	mutex_lock(&panel->panel_lock);
	/* retry: the tx path can drop a command with -ENOMEM while the dsi
	 * controller is being reconfigured, and a single miss here leaves the
	 * panel in the normal gamma for the rest of the doze */
	for (tries = HOSHIKV_FOD_FPS_TRIES; tries > 0; tries--) {
		rc = mi_dsi_panel_aod_to_normal_optimize_locked(panel,
				false);
		if (!rc)
			break;
		usleep_range(2000, 4000);
	}
	/* the aod enter is what makes the idle doze look right, so it must go
	 * out even when the vendor disable above refused */
	aod_rc = mi_dsi_panel_fod_aod_switch_locked(panel, true);
	if (aod_rc == -ENOTSUPP)
		aod_rc = 0;
	mutex_unlock(&panel->panel_lock);

	if (aod_rc)
		DISP_ERROR("hoshikv-fod: aod enter failed (rc=%d)\n", aod_rc);

	if (rc)
		DISP_ERROR("hoshikv-fod: doze NOLP off failed (rc=%d),"
			" fallback lp doze\n", rc);

	if (rc && !aod_rc) {
		/* vendor disable refused (state out of sync): go back to lp
		 * doze by hand so the panel does not stay stuck in normal mode */
		if (prev_mode == SDE_MODE_DPMS_LP2)
			dsi_panel_set_lp2(panel);
		else
			dsi_panel_set_lp1(panel);
	}

	mi_dsi_panel_hoshikv_doze_fps(panel, false);
	DISP_INFO("hoshikv-fod: doze NOLP off, back to doze 30Hz (mode=%d)\n",
		panel->power_mode);
}

/*
 * The DPMS path owns the panel when the screen really turns on/off: just drop
 * the NOLP bookkeeping, never send LP tx from here.
 *
 * hoshikv: keep_fod=true when this transition belongs to an in-flight FOD touch
 * (the doze->normal walk SDM does for a press). It is NOT a screen-off, so the
 * 3s doze-120Hz hold must survive it. Only a real DPMS_ON/OFF hands the panel
 * back and may cancel the hold.
 */
void mi_disp_lhbm_fod_doze_nolp_abort(struct dsi_display *display,
		bool keep_fod)
{
	struct disp_lhbm_fod *lhbm_fod;
	struct dsi_panel *panel;

	if (!display || !display->panel)
		return;

	lhbm_fod = mi_get_disp_lhbm_fod(
		mi_get_disp_id(display->display_type));
	if (!lhbm_fod)
		return;

	panel = display->panel;

	/*
	 * hoshikv: bailed out on fod_nolp_on alone, which the SDM-doze path never
	 * sets (it only sets fod_sdm_doze). So after a real DPMS_OFF this returned
	 * without clearing fod_sdm_doze, and the next watch_disable() ran
	 * hoshikv_fod_doze_nolp_leave() for a hold that was never armed -- that is
	 * what dropped the panel back to 30Hz under a lit fingerprint, ~400us
	 * after the press. Reset both ownership flags.
	 */
	if (!lhbm_fod->fod_nolp_on && !lhbm_fod->fod_sdm_doze)
		return;

	/*
	 * hoshikv: this used to tear the hold down unconditionally, and the
	 * FOD doze->normal DPMS walk fired it ~40ms after the panel went to
	 * 120Hz, so doze 120Hz never lasted longer than that.
	 *
	 * Do NOT gate on fod_watch_en: the watch kthread calls v() and clears it
	 * on every transition, so by the time this DPMS walk arrives it is
	 * already 0 and a guard on it never fires. The armed hold with a
	 * deadline still in the future is the reliable signal.
	 */
	if (keep_fod && lhbm_fod->fod_hold_armed &&
	    time_before(jiffies, lhbm_fod->fod_hold_deadline)) {
		lhbm_fod->fod_nolp_on = 0;
		lhbm_fod->fod_nolp_prev_mode = 0;
		DISP_INFO("hoshikv-fod: dpms during FOD hold, keeping doze 120Hz"
			" (left=%ums)\n",
			jiffies_to_msecs(lhbm_fod->fod_hold_deadline -
				jiffies));
		return;
	}

	lhbm_fod->fod_nolp_on = 0;
	lhbm_fod->fod_nolp_prev_mode = 0;
	lhbm_fod->fod_hold_armed = 0;
	lhbm_fod->fod_sdm_doze = 0;
	/* the real power path owns the panel from here; drop the vendor
	 * aod_to_normal flag too or dsi_panel_set_lp2() would keep skipping */
	panel->mi_cfg.aod_to_normal_statue = false;
	/* no async release: this runs under mi_cfg.doze_lock from the DPMS
	 * transition, and the worker would race it back into doze */
	cancel_delayed_work(&lhbm_fod->fod_hold_work);
	DISP_INFO("hoshikv-fod: doze NOLP aborted by dpms path\n");
}

/* mirror the touch state to hoshikv_fod_state and poke the lib's poll. */
static void hoshikv_fod_publish(struct disp_lhbm_fod *lhbm_fod, int on)
{
	struct device *dev = lhbm_fod->fod_sysdev;

	atomic_set(&lhbm_fod->fod_press, !!on);
	atomic_set(&lhbm_fod->fod_state_pub, !!on);
	if (dev)
		sysfs_notify(&dev->kobj, NULL, HOSHIKV_FOD_STATE_ATTR);
}

/* mirror the lib's hbm_on(): pull DC dimming off so it can't grey the HBM
 * region during fingerprint capture. no-op unless the panel has DC enabled. */
static int hoshikv_fod_set_dc(struct disp_lhbm_fod *lhbm_fod, bool on)
{
	struct disp_feature_ctl ctl;
	struct dsi_panel *panel = lhbm_fod->display->panel;

	memset(&ctl, 0, sizeof(ctl));
	ctl.feature_id = DISP_FEATURE_DC;
	ctl.feature_val = on ? FEATURE_ON : FEATURE_OFF;
	return mi_dsi_panel_set_disp_param(panel, &ctl);
}

static void hoshikv_fod_press(struct disp_lhbm_fod *lhbm_fod)
{
	struct dsi_display *display = lhbm_fod->display;
	struct dsi_panel *panel = display->panel;
	int disp_id = mi_get_disp_id(display->display_type);
	bool in_doze;
	int tries;

	in_doze = (panel->power_mode == SDE_MODE_DPMS_LP1 ||
		   panel->power_mode == SDE_MODE_DPMS_LP2);

	if (!hoshikv_fod_panel_stable(panel)) {
		/* Screen-off -> FOD: the touch can land while the panel is still
		 * coming up to doze, or before doze is armed at all. Dropping the
		 * press here is what produced "fod anim but no light": the
		 * animation is userspace, the HBM is us. Remember the press and
		 * let the watch loop re-run it as soon as doze is real. */
		lhbm_fod->fod_press_pending = 1;
		lhbm_fod->fod_press_deadline =
			jiffies + msecs_to_jiffies(HOSHIKV_FOD_HOLD_MS);
		DISP_INFO("hoshikv-fod: press deferred, panel not stable"
			" (mode=%d)\n", panel->power_mode);
		return;
	}

	lhbm_fod->fod_press_pending = 0;

	if (panel->mi_cfg.dc_feature_enable &&
	    panel->mi_cfg.feature_val[DISP_FEATURE_DC] == FEATURE_ON) {
		if (!hoshikv_fod_set_dc(lhbm_fod, false))
			lhbm_fod->fod_dc_restore = 1;
	}

	/* FOD press in doze: leave LP first (doze NOLP -> panel at full fps),
	 * and that MUST have landed before the HBM is queued, otherwise the
	 * animation runs on the 30Hz doze frames (slow motion) and the first lit
	 * frame is one doze frame late. Retry a few times: the cmd can lose a
	 * race against a still-queued LHBM off from the previous press.
	 * If NOLP never lands we still inject the HBM, just in LP doze. */
	if (in_doze) {
		/*
		 * Every doze flavour is SDM's: static AOD, seamless AOD,
		 * screen-off AOD, and the fingerprint ui itself. Ask SDM for the
		 * doze->normal transition once, and arm the 3s hold so the panel
		 * stays out of LP for as long as the finger is doing something.
		 * From here the driver only lights HBM; fps/brightness and the
		 * aod->normal restore belong to SDM until the hold expires.
		 */
		int nolp_rc = 0;

		for (tries = HOSHIKV_FOD_FPS_TRIES; tries > 0; tries--) {
			nolp_rc = hoshikv_fod_doze_nolp_enter(lhbm_fod);
			if (!nolp_rc)
				break;
			usleep_range(2000, 4000);
		}
		if (nolp_rc) {
			/* hoshikv: the panel never left doze. Queuing HBM here lit
			 * 1000nit on a panel still sitting in the lp aod gamma, which
			 * is the "fod anim but grey/green panel" case. Remember the
			 * press and let the watch loop retry once doze is real; the
			 * HBM goes in then, after 120Hz is actually up. */
			lhbm_fod->fod_press_pending = 1;
			lhbm_fod->fod_press_deadline =
				jiffies + msecs_to_jiffies(HOSHIKV_FOD_HOLD_MS);
			DISP_ERROR("hoshikv-fod: doze NOLP latch failed"
				" (rc=%d), HBM deferred\n", nolp_rc);
			return;
		}

		/* (re)start the 3s hold so back-to-back touches keep doze NOLP */
		hoshikv_fod_hold_arm(lhbm_fod);
	}

	DISP_INFO("hoshikv-fod: press detected\n");
	mi_disp_set_local_hbm(disp_id, LHBM_TARGET_BRIGHTNESS_WHITE_1000NIT);
}

static void hoshikv_fod_release(struct disp_lhbm_fod *lhbm_fod)
{
	int disp_id = mi_get_disp_id(lhbm_fod->display->display_type);

	/* finger up cancels a deferred press: never light HBM after lift */
	lhbm_fod->fod_press_pending = 0;

	/* HBM off, but keep doze NOLP until the hold expires */
	hoshikv_fod_hold_arm(lhbm_fod);
	lhbm_fod->fod_fps_best_effort = 0;

	if (lhbm_fod->fod_dc_restore) {
		hoshikv_fod_set_dc(lhbm_fod, true);
		lhbm_fod->fod_dc_restore = 0;
	}

	/* finger up -> the lhbm thread sends MI_DISP_EVENT_FOD (LOCAL_HBM_UI_NONE)
	 * for the HAL when mi_disp_set_local_hbm(..OFF_FINGER_UP) below lands; no
	 * duplicate notify here. doze keeps NOLP via the hold until it expires. */
	DISP_INFO("hoshikv-fod: release detected\n");
	mi_disp_set_local_hbm(disp_id, LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
}

/*
 * hoshikv: release doze NOLP once the hold expires. This runs off
 * schedule_delayed_work rather than the watch kthread because v() stops that
 * kthread, and a hold armed right before the stop would otherwise never expire.
 */
static void hoshikv_fod_hold_expire_work(struct work_struct *work)
{
	struct disp_lhbm_fod *lhbm_fod =
		container_of(to_delayed_work(work), struct disp_lhbm_fod,
				fod_hold_work);

	/* a press/release may have re-armed the hold while we were queued */
	if (!lhbm_fod->fod_hold_armed)
		return;

	if (time_before(jiffies, lhbm_fod->fod_hold_deadline)) {
		unsigned long left = lhbm_fod->fod_hold_deadline - jiffies;

		mod_delayed_work(system_wq, &lhbm_fod->fod_hold_work,
				msecs_to_jiffies(left) + 1);
		return;
	}

	lhbm_fod->fod_hold_armed = 0;
	DISP_INFO("hoshikv-fod: hold timeout, back to doze 30Hz\n");
	hoshikv_fod_doze_nolp_leave(lhbm_fod);
	mi_dsi_panel_hoshikv_doze_fps(lhbm_fod->display->panel, false);
}

/* arm the 3s hold and make sure the expiry runs even if the kthread stops. */
static void hoshikv_fod_hold_arm(struct disp_lhbm_fod *lhbm_fod)
{
	lhbm_fod->fod_hold_deadline =
		jiffies + msecs_to_jiffies(HOSHIKV_FOD_HOLD_MS);
	lhbm_fod->fod_hold_armed = 1;
	mod_delayed_work(system_wq, &lhbm_fod->fod_hold_work,
			msecs_to_jiffies(HOSHIKV_FOD_HOLD_MS));
}

/* keep doze NOLP until the deadline, then fall back to doze 30Hz. */
static void hoshikv_fod_hold_tick(struct disp_lhbm_fod *lhbm_fod)
{
	/*
	 * A press that arrived before doze was ready gets one more chance
	 * every tick, otherwise it waits for the next finger lift.
	 *
	 * hoshikv: the replay must be allowed to arm the hold. hoshikv_fod_press()
	 * clears fod_press_pending as soon as the panel is stable, so this runs
	 * at most once per deferred press and cannot extend the deadline in a
	 * loop -- restoring the old deadline here would just discard the hold the
	 * replay legitimately started.
	 */
	if (lhbm_fod->fod_press_pending) {
		if (time_after_eq(jiffies, lhbm_fod->fod_press_deadline)) {
			lhbm_fod->fod_press_pending = 0;
			DISP_INFO("hoshikv-fod: deferred press expired\n");
		} else if (hoshikv_fod_panel_stable(lhbm_fod->display->panel)) {
			DISP_INFO("hoshikv-fod: replaying deferred press\n");
			hoshikv_fod_press(lhbm_fod);
			return;
		}
	}

	if (!lhbm_fod->fod_hold_armed)
		return;

	/*
	 * hoshikv: hoshikv_fod_hold_expire_work() owns the actual release, since it
	 * keeps running when v() stops this kthread. Only nudge it if the deadline
	 * is already past and the work is not pending.
	 */
	if (time_after_eq(jiffies, lhbm_fod->fod_hold_deadline) &&
	    !delayed_work_pending(&lhbm_fod->fod_hold_work))
		mod_delayed_work(system_wq, &lhbm_fod->fod_hold_work, 0);
}

/* woken by the touch node's kernfs waitqueue (kernfs_notify -> wake_up). */
static int hoshikv_fod_poll_wqfunc_entry(struct wait_queue_entry *entry,
		unsigned int mode, int flags, void *key)
{
	struct disp_lhbm_fod *lhbm_fod = container_of(entry,
			struct disp_lhbm_fod, fod_poll_entry);

	atomic_set(&lhbm_fod->fod_poll_event, 1);
	wake_up_interruptible(&lhbm_fod->fod_poll_wq);
	return 0;
}

/*
 * poll_table probed by the touch node's ->poll(). kernfs_fop_poll ->
 * kernfs_generic_poll calls poll_wait(..., &on->poll, wait), handing us the
 * node's waitqueue. We inject a single entry so our kthread is woken on each
 * sysfs_notify instead of having to poll the value.
 */
static void hoshikv_fod_poll_qproc(struct file *fp,
		wait_queue_head_t *wq, struct poll_table_struct *pt)
{
	struct disp_lhbm_fod *lhbm_fod = container_of(pt,
			struct disp_lhbm_fod, fod_poll_pt);

	if (!lhbm_fod->fod_poll_hooked && wq) {
		lhbm_fod->fod_poll_parent = wq;
		lhbm_fod->fod_poll_hooked = true;
		add_wait_queue(wq, &lhbm_fod->fod_poll_entry);
	}
}

static int hoshikv_fod_touch_open(struct disp_lhbm_fod *lhbm_fod)
{
	struct file *fp;

	if (lhbm_fod->fod_touch_file)
		return 0;

	fp = filp_open(HOSHIKV_FOD_TOUCH_NODE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		DISP_INFO("hoshikv-fod: open %s failed (%ld)\n",
				HOSHIKV_FOD_TOUCH_NODE, PTR_ERR(fp));
		return PTR_ERR(fp);
	}

	lhbm_fod->fod_touch_file = fp;
	lhbm_fod->fod_poll_hooked = false;
	lhbm_fod->fod_poll_parent = NULL;
	atomic_set(&lhbm_fod->fod_poll_event, 0);

	/* register our entry + establish the of->event baseline */
	if (fp->f_op && fp->f_op->poll)
		fp->f_op->poll(fp, &lhbm_fod->fod_poll_pt);

	DISP_INFO("hoshikv-fod: touch node opened\n");
	return 0;
}

static void hoshikv_fod_touch_close(struct disp_lhbm_fod *lhbm_fod)
{
	struct file *fp = lhbm_fod->fod_touch_file;

	if (!fp)
		return;

	/* drop our wq entry *before* fput so the kernfs waitqueue can vanish */
	if (lhbm_fod->fod_poll_hooked && lhbm_fod->fod_poll_parent) {
		remove_wait_queue(lhbm_fod->fod_poll_parent,
				&lhbm_fod->fod_poll_entry);
		lhbm_fod->fod_poll_hooked = false;
		lhbm_fod->fod_poll_parent = NULL;
	}
	fput(fp);
	lhbm_fod->fod_touch_file = NULL;
	atomic_set(&lhbm_fod->fod_poll_event, 0);
	DISP_INFO("hoshikv-fod: touch node closed\n");
}

/* consume the one-shot fod_press_status value (1 = pressed, 0 = released). */
static int hoshikv_fod_touch_read(struct disp_lhbm_fod *lhbm_fod)
{
	struct file *fp = lhbm_fod->fod_touch_file;
	char buf[8];
	loff_t pos = 0;
	ssize_t n;

	if (!fp)
		return 0;

	fp->f_pos = 0;
	n = kernel_read(fp, buf, sizeof(buf) - 1, &pos);
	if (n <= 0) {
		DISP_INFO("hoshikv-fod: touch read failed (%ld)\n", n);
		return 0;
	}

	return (buf[0] == '1');
}

static int mi_disp_lhbm_fod_watch_thread_fn(void *arg)
{
	struct disp_lhbm_fod *lhbm_fod = (struct disp_lhbm_fod *)arg;
	int cur;

	while (!kthread_should_stop()) {
		/* parked; only a SET_FOD_MODE ioctl (enable) wakes us */
		if (!atomic_read(&lhbm_fod->fod_watch_en)) {
			mutex_lock(&lhbm_fod->fod_touch_lock);
			hoshikv_fod_touch_close(lhbm_fod);
			mutex_unlock(&lhbm_fod->fod_touch_lock);

			wait_event_interruptible(lhbm_fod->fod_watch_wq,
					atomic_read(&lhbm_fod->fod_watch_en) ||
					kthread_should_stop());
			continue;
		}

		mutex_lock(&lhbm_fod->fod_touch_lock);
		if (!lhbm_fod->fod_touch_file)
			hoshikv_fod_touch_open(lhbm_fod);
		mutex_unlock(&lhbm_fod->fod_touch_lock);

		if (!lhbm_fod->fod_touch_file) {
			/* touch node unavailable; back off and retry */
			wait_event_interruptible_timeout(lhbm_fod->fod_watch_wq,
					kthread_should_stop() ||
					!atomic_read(&lhbm_fod->fod_watch_en),
					msecs_to_jiffies(HOSHIKV_FOD_WAIT_MS));
			continue;
		}

		/* prime the baseline so only real edges trigger press/release */
		mutex_lock(&lhbm_fod->fod_touch_lock);
		cur = hoshikv_fod_touch_read(lhbm_fod);
		mutex_unlock(&lhbm_fod->fod_touch_lock);
		/* A finger can already be down at arm time (k() while touching). That
		 * is a real FOD press, so treat it as one in every power state:
		 * hoshikv_fod_press() itself does doze120-before-HBM in doze and
		 * leaves the rate alone when awake. Skipping it here is what made
		 * the first touch after arming sometimes never light HBM. */
		if (cur) {
			hoshikv_fod_publish(lhbm_fod, cur);
			hoshikv_fod_press(lhbm_fod);
		}

		/* wait for press/release notifications on the touch node */
		while (!kthread_should_stop() &&
				atomic_read(&lhbm_fod->fod_watch_en)) {
			int last = cur;
			__poll_t revents;

			mutex_lock(&lhbm_fod->fod_touch_lock);
			if (!lhbm_fod->fod_touch_file) {
				mutex_unlock(&lhbm_fod->fod_touch_lock);
				break;
			}

			atomic_set(&lhbm_fod->fod_poll_event, 0);
			revents = lhbm_fod->fod_touch_file->f_op->poll(
					lhbm_fod->fod_touch_file,
					&lhbm_fod->fod_poll_pt);
			if (revents & (POLLPRI | POLLERR | POLLIN))
				cur = hoshikv_fod_touch_read(lhbm_fod);
			mutex_unlock(&lhbm_fod->fod_touch_lock);

			/* act only on a value transition; poll may stay ready even
			 * when the value is unchanged, so this is what keeps the
			 * kthread from busy-looping on a flat value.
			 *
			 * hoshikv: the transition is handled BEFORE hold_tick(). The
			 * other order let the expiry worker fire on the *old* deadline
			 * in the same tick that a re-touch was re-arming it, so the
			 * worker could drop doze 120Hz right under a finger that was
			 * still down. */
			if (cur != last) {
				hoshikv_fod_publish(lhbm_fod, cur);
				if (cur)
					hoshikv_fod_press(lhbm_fod);
				else
					hoshikv_fod_release(lhbm_fod);
			}

			hoshikv_fod_hold_tick(lhbm_fod);

			/* bounded sleep floor (awake nodes can report ready forever) */
			wait_event_interruptible_timeout(lhbm_fod->fod_poll_wq,
					kthread_should_stop() ||
					!atomic_read(&lhbm_fod->fod_watch_en) ||
					atomic_read(&lhbm_fod->fod_poll_event),
					msecs_to_jiffies(
						lhbm_fod->fod_press_pending ? 5 :
						(lhbm_fod->fod_hold_armed ?
						 HOSHIKV_FOD_HOLD_REARM_MS :
						 HOSHIKV_FOD_WAIT_MS)));
			if (!atomic_read(&lhbm_fod->fod_watch_en))
				break;
		}
	}

	return 0;
}

int mi_disp_lhbm_fod_watch_enable(struct disp_feature *df, int disp_id,
		bool enable)
{
	struct disp_lhbm_fod *lhbm_fod = mi_get_disp_lhbm_fod(disp_id);

	if (!lhbm_fod) {
		DISP_ERROR("%s invalid lhbm_fod ptr\n", get_disp_id_name(disp_id));
		return -EINVAL;
	}

	if (enable) {
		atomic_set(&lhbm_fod->fod_watch_en, 1);
		atomic_set(&lhbm_fod->fod_press, 0);
		atomic_set(&lhbm_fod->fod_poll_event, 0);
		lhbm_fod->fod_hold_armed = 0;
		lhbm_fod->fod_fps_last_notify = jiffies;
		lhbm_fod->fod_fps_best_effort = 0;

		/* SET_FOD_MODE(1) = k(): arm watch only. Reset animation to off here
		 * so the first v()/k() cycle starts from a clean false baseline. */
		hoshikv_fod_publish(lhbm_fod, 0);
	} else {
		atomic_set(&lhbm_fod->fod_watch_en, 0);
		atomic_set(&lhbm_fod->fod_press, 0);
		if (lhbm_fod->fod_dc_restore) {
			hoshikv_fod_set_dc(lhbm_fod, true);
			lhbm_fod->fod_dc_restore = 0;
		}
		mi_disp_set_local_hbm(disp_id,
				LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);

		/*
		 * hoshikv: v() arrives while the finger is still down (the lib drops
		 * its FOD UI on every v()), so releasing doze NOLP here
		 * unconditionally killed the 3s hold mid-press and put the panel back
		 * to 30Hz under a lit fingerprint. Keep the hold and let
		 * hoshikv_fod_hold_expire_work() expire it, which also survives the
		 * kthread stop below.
		 */
		if (lhbm_fod->fod_hold_armed &&
		    time_before(jiffies, lhbm_fod->fod_hold_deadline)) {
			/* hoshikv: the 30Hz restore is what actually drops doze
			 * 120Hz, so it must not run while the hold is live. Only
			 * hoshikv_fod_hold_expire_work() may put the panel back to
			 * 30Hz, and it does that 3s after the last touch. */
			DISP_INFO("hoshikv-fod: v() during hold, keeping"
				" doze 120Hz until timeout\n");
		} else {
			lhbm_fod->fod_hold_armed = 0;
			cancel_delayed_work_sync(&lhbm_fod->fod_hold_work);
			/* v() drops the FOD UI: give doze NOLP back */
			hoshikv_fod_doze_nolp_leave(lhbm_fod);
			/* own 30Hz restore, no Xiaomi doze_brightness */
			mi_dsi_panel_hoshikv_doze_fps(lhbm_fod->display->panel,
					false);
		}
		/* SET_FOD_MODE(0) = v(): do NOT publish 0 here. The fingerprint
		 * animation (onFpTouch) must stay true until finger-up. Publishing 0
		 * on v() cuts the fod animation mid-sequence at the lockscreen. The
		 * state is only reset to 0 by the next k() or by an actual release. */
	}

	wake_up_interruptible(&lhbm_fod->fod_watch_wq);
	wake_up_interruptible(&lhbm_fod->fod_poll_wq);
	DISP_INFO("hoshikv-fod: watch %s\n", enable ? "enable" : "disable");
	return 0;
}

int mi_disp_lhbm_fod_state_pub_get(int disp_id)
{
	struct disp_lhbm_fod *lhbm_fod = mi_get_disp_lhbm_fod(disp_id);

	if (!lhbm_fod)
		return -EINVAL;

	return atomic_read(&lhbm_fod->fod_state_pub);
}

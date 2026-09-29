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
			if (is_aod_and_panel_initialized(panel) &&
				(mi_cfg->panel_state == PANEL_STATE_DOZE_HIGH
				||mi_cfg->panel_state == PANEL_STATE_DOZE_LOW))
				ctl.feature_val = LOCAL_HBM_HLPM_WHITE_1000NIT;
			else
				ctl.feature_val = LOCAL_HBM_NORMAL_WHITE_1000NIT;
		} else if (lhbm_value == LHBM_TARGET_BRIGHTNESS_WHITE_110NIT) {
			if (is_aod_and_panel_initialized(panel) &&
				(mi_cfg->panel_state == PANEL_STATE_DOZE_HIGH
				||mi_cfg->panel_state == PANEL_STATE_DOZE_LOW))
				ctl.feature_val = LOCAL_HBM_HLPM_WHITE_110NIT;
			else
				ctl.feature_val = LOCAL_HBM_NORMAL_WHITE_110NIT;
		} else {
			DISP_ERROR("invalid target_brightness = %d\n", lhbm_fod->target_brightness);
		}
		break;
	case LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP:
		ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
		if (is_aod_and_panel_initialized(panel))
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL_BACKLIGHT_RESTORE;
		else
			ctl.feature_val = LOCAL_HBM_OFF_TO_NORMAL;
		break;
	case LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP:
		ctl.feature_id = DISP_FEATURE_LOCAL_HBM;
		if (is_aod_and_panel_initialized(panel)) {
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
		if ((mi_get_panel_id_by_dsi_panel(lhbm_fod->display->panel) == N16T_PANEL_PA ||
			mi_get_panel_id_by_dsi_panel(lhbm_fod->display->panel) == N16T_PANEL_PB)
			&& is_aod_and_panel_initialized(lhbm_fod->display->panel)) {
			/* Notify switch to fod fps */
			if (lhbm_setting_event.lhbm_value != LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP &&
				lhbm_setting_event.lhbm_value != LHBM_TARGET_BRIGHTNESS_OFF_AUTH_STOP) {
				rc = mi_disp_lhbm_fod_event_notify(lhbm_fod, FOD_EVENT_FPS);
				if (rc == -NEED_UPDATE_TO_FOD_FPS) {
					mi_disp_lhbm_fod_allow_tx_lhbm(lhbm_fod->display, false);
					DISP_INFO("Stop to allow tx lhbm, wait to swtich fod fps!");
					spin_unlock_irqrestore(&lhbm_fod->spinlock, flag);
					continue;
				}
			}
		}
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
 *     libhoshikv k()/v() -> fod_mode_set(fd, 1/0). No auto-arm at AOD entry and
 *     no polling of /dev/xiaomi-touch mode 10.
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

	if (panel->power_mode != SDE_MODE_DPMS_LP1 &&
		panel->power_mode != SDE_MODE_DPMS_LP2) {
		DISP_INFO("hoshikv-fod: panel power_mode=%d not stable,"
			" skip HBM inj\n", panel->power_mode);
		return false;
	}

	return true;
}

/* doze 30Hz -> doze 120Hz: ask HAL, then wait for the rate to land. */
static int hoshikv_fod_force_fod_fps(struct disp_lhbm_fod *lhbm_fod)
{
	struct dsi_panel *panel = lhbm_fod->display->panel;
	unsigned long deadline;
	u32 rate = 0;
	int rc = 0;

	rc = mi_disp_lhbm_fod_event_notify(lhbm_fod, FOD_EVENT_FPS);
	if (rc != -NEED_UPDATE_TO_FOD_FPS)
		return 0;

	deadline = jiffies + msecs_to_jiffies(HOSHIKV_FOD_FPS_WAIT_MS);
	do {
		rate = panel->cur_mode->timing.refresh_rate;
		if (rate >= NEED_UPDATE_TO_FOD_FPS)
			break;
		usleep_range(2000, 3000);
	} while (time_before(jiffies, deadline));

	rate = panel->cur_mode->timing.refresh_rate;
	if (rate < NEED_UPDATE_TO_FOD_FPS) {
		DISP_INFO("hoshikv-fod: fod fps wait timeout, rate=%d\n", rate);
		return -ETIMEDOUT;
	}

	DISP_INFO("hoshikv-fod: fod fps ready, rate=%d\n", rate);
	return 0;
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

static void hoshikv_fod_press(struct disp_lhbm_fod *lhbm_fod)
{
	struct dsi_display *display = lhbm_fod->display;
	int disp_id = mi_get_disp_id(display->display_type);

	if (!hoshikv_fod_panel_stable(display->panel))
		return;

	/* route to doze 120Hz *before* the HBM goes on */
	if (hoshikv_fod_force_fod_fps(lhbm_fod))
		DISP_INFO("hoshikv-fod: injecting HBM without confirmed fod fps\n");

	/* (re)start the 3s hold so back-to-back touches keep doze at 120Hz */
	lhbm_fod->fod_hold_deadline =
		jiffies + msecs_to_jiffies(HOSHIKV_FOD_HOLD_MS);
	lhbm_fod->fod_hold_armed = 1;

	DISP_INFO("hoshikv-fod: press detected\n");
	mi_disp_set_local_hbm(disp_id, LHBM_TARGET_BRIGHTNESS_WHITE_1000NIT);
}

static void hoshikv_fod_release(struct disp_lhbm_fod *lhbm_fod)
{
	int disp_id = mi_get_disp_id(lhbm_fod->display->display_type);

	/* HBM off, but keep doze at 120Hz until the hold expires */
	lhbm_fod->fod_hold_deadline =
		jiffies + msecs_to_jiffies(HOSHIKV_FOD_HOLD_MS);
	lhbm_fod->fod_hold_armed = 1;

	DISP_INFO("hoshikv-fod: release detected\n");
	mi_disp_set_local_hbm(disp_id, LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
}

/* hold doze at 120Hz until the deadline, then fall back to doze 30Hz. */
static void hoshikv_fod_hold_tick(struct disp_lhbm_fod *lhbm_fod)
{
	if (!lhbm_fod->fod_hold_armed)
		return;

	if (time_after_eq(jiffies, lhbm_fod->fod_hold_deadline)) {
		lhbm_fod->fod_hold_armed = 0;
		DISP_INFO("hoshikv-fod: hold timeout, back to doze 30Hz\n");
		mi_disp_lhbm_fod_event_notify(lhbm_fod, FOD_EVENT_UP);
		mi_dsi_display_set_doze_brightness(lhbm_fod->display,
				DOZE_BRIGHTNESS_HBM);
		return;
	}

	/* keep re-asserting the fod rate while inside the hold window */
	if (time_after_eq(jiffies, lhbm_fod->fod_fps_last_notify +
			msecs_to_jiffies(HOSHIKV_FOD_HOLD_REARM_MS))) {
		lhbm_fod->fod_fps_last_notify = jiffies;
		mi_disp_lhbm_fod_event_notify(lhbm_fod, FOD_EVENT_FPS);
	}
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

		/* consume any press that landed before the node was opened */
		mutex_lock(&lhbm_fod->fod_touch_lock);
		cur = hoshikv_fod_touch_read(lhbm_fod);
		mutex_unlock(&lhbm_fod->fod_touch_lock);
		if (cur) {
			hoshikv_fod_publish(lhbm_fod, cur);
			hoshikv_fod_press(lhbm_fod);
		}

		/* wait for press/release notifications on the touch node */
		while (!kthread_should_stop() &&
				atomic_read(&lhbm_fod->fod_watch_en)) {
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
			if (revents & (POLLPRI | POLLERR | POLLIN)) {
				cur = hoshikv_fod_touch_read(lhbm_fod);
				mutex_unlock(&lhbm_fod->fod_touch_lock);

				hoshikv_fod_publish(lhbm_fod, cur);
				if (cur)
					hoshikv_fod_press(lhbm_fod);
				else
					hoshikv_fod_release(lhbm_fod);
				break;
			}
			mutex_unlock(&lhbm_fod->fod_touch_lock);

			hoshikv_fod_hold_tick(lhbm_fod);

			wait_event_interruptible_timeout(lhbm_fod->fod_poll_wq,
					kthread_should_stop() ||
					!atomic_read(&lhbm_fod->fod_watch_en) ||
					atomic_read(&lhbm_fod->fod_poll_event),
					msecs_to_jiffies(
						lhbm_fod->fod_hold_armed ?
						HOSHIKV_FOD_HOLD_REARM_MS :
						HOSHIKV_FOD_WAIT_MS));
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
	} else {
		atomic_set(&lhbm_fod->fod_watch_en, 0);
		atomic_set(&lhbm_fod->fod_press, 0);
		lhbm_fod->fod_hold_armed = 0;
		mi_disp_set_local_hbm(disp_id,
				LHBM_TARGET_BRIGHTNESS_OFF_FINGER_UP);
		hoshikv_fod_publish(lhbm_fod, 0);
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

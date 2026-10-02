/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2020 XiaoMi, Inc. All rights reserved.
 */

#ifndef _MI_DISP_LHBM_H_
#define _MI_DISP_LHBM_H_

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/mutex.h>
#include <linux/device.h>

#include "dsi_panel.h"
#include "dsi_display.h"
#include "mi_disp_feature.h"
#include "mi_sde_connector.h"

#define NEED_UPDATE_TO_FOD_FPS 120

/*
 * hoshikv FOD-HBM
 *   - notification-driven: the fod_watch kthread is armed/stopped ONLY by
 *     MI_DISP_IOCTL_SET_FOD_MODE (disp_feature_req feature_val), matching
 *     libhoshikv (k()/v() -> fod_mode_set 1/0). No auto-arm at doze entry and
 *     no polling of /dev/xiaomi-touch mode 10.
 *   - while armed the kthread is the SOLE reader of
 *     /sys/class/touch/touch_dev/fod_press_status: it waits for the node's
 *     kernfs notification (POLLPRI) instead of polling, then consumes the
 *     oneshot value. Touch `notify_oneshot_sensor(ONESHOT_SENSOR_FOD_PRESS, 1)`
 *     on press and (..., 0) on release both sysfs_notify -> hoshikv_fod_poll_*
 *     lets us see per-edge press/release without patching the touch driver.
 *   - press -> mirror hoshikv_fod_state to 1, sysfs_notify (lib's
 *     poll(POLLPRI) wakes onFpTouch(true)), doze 120Hz + local HBM (HLPM)
 *   - release -> mirror 0 + sysfs_notify (onFpTouch(false)), HBM off,
 *     hold doze 120Hz until HOSHIKV_FOD_HOLD_MS then drop to doze 30Hz
 *   - re-press -> reset hold timer (spam FOD tanpa bolak-balik 30/120)
 */
#define HOSHIKV_FOD_HOLD_MS		3000
#define HOSHIKV_FOD_HOLD_REARM_MS	500
#define HOSHIKV_FOD_FPS_WAIT_MS		400
#define HOSHIKV_FOD_WAIT_MS		250
/* doze 30->120 must latch before the FOD HBM region lights; the DOZE_HBM cmd
 * is skipped while a previous press still has its LHBM region marked on, so
 * retry this many times (2-4ms apart) before queueing the HBM. */
#define HOSHIKV_FOD_FPS_TRIES		10
#define HOSHIKV_FOD_TOUCH_NODE		"/sys/class/touch/touch_dev/fod_press_status"
#define HOSHIKV_FOD_STATE_ATTR		"hoshikv_fod_state"

enum {
	FOD_EVENT_UP = 0,
	FOD_EVENT_DOWN = 1,
	FOD_EVENT_FPS = 2,
	FOD_EVENT_MAX
};

enum mi_panel_op_code {
	MI_FOD_HBM_ON = 0,
	MI_FOD_HBM_OFF,
	MI_FOD_AOD_TO_NORMAL,
	MI_FOD_NORMAL_TO_AOD,
};

enum fod_ui_ready_state {
	LOCAL_HBM_UI_NONE = 0,
	GLOBAL_FOD_HBM_OVERLAY = BIT(0),
	GLOBAL_FOD_ICON = BIT(1),
	FOD_LOW_BRIGHTNESS_CAPTURE = BIT(2),
	LOCAL_HBM_UI_READY  = BIT(3),
	LOCAL_HBM_NEED_UPDATE_TO_FOD_FPS = BIT(4)
};

struct disp_lhbm_fod {
	struct dsi_display *display;

	struct task_struct *fod_thread;
	wait_queue_head_t fod_pending_wq;

	struct list_head event_list;
	spinlock_t spinlock;

	atomic_t allow_tx_lhbm;

	struct mi_layer_flags layer_flags;

	atomic_t target_brightness;

	atomic_t disp_off_target_brightness;

	/* hoshikv FOD watch (notification-driven) */
	struct task_struct *fod_watch_thread;
	wait_queue_head_t fod_watch_wq;
	struct mutex fod_touch_lock;	/* guards the touch node file + poll */
	struct file *fod_touch_file;	/* filp on fod_press_status while armed */
	struct poll_table_struct fod_poll_pt;	/* poll hook: _qproc = ..._qproc */
	bool fod_poll_hooked;
	wait_queue_head_t *fod_poll_parent;	/* touch kernfs on->poll */
	wait_queue_entry_t fod_poll_entry;	/* injected into fod_poll_parent */
	atomic_t fod_poll_event;	/* 1 = touch node notified us */
	wait_queue_head_t fod_poll_wq;	/* woken by fod_poll_entry.func */
	struct device *fod_sysdev;	/* disp_display device for sysfs_notify */
	atomic_t fod_watch_en;
	atomic_t fod_press;		/* current press state from touch */
	atomic_t fod_state_pub;		/* value exported to sysfs */
	int fod_hold_armed;
	unsigned long fod_hold_deadline;
	unsigned long fod_fps_last_notify;
int fod_fps_best_effort;	/* 1 = HAL didn't land 120Hz, inject HBM anyway */
	int fod_dc_restore;		/* 1 = DC pulled off for HBM, restore on release */
	int fod_nolp_on;		/* 1 = doze NOLP (full fps) owned by the FOD press */
	int fod_nolp_prev_mode;	/* power_mode we faked away when entering NOLP */
	int fod_sdm_doze;	/* 1 = SDM owns doze: driver must not touch
				 * fps/brightness at all */
	int fod_press_pending;	/* 1 = press seen, panel not in doze yet */
	unsigned long fod_press_deadline;
	/*
	 * hoshikv: hoshikv_fod_hold_tick() only runs inside the watch kthread, and
	 * v() (SET_FOD_MODE 0) stops that kthread. A hold armed just before the
	 * stop would then never expire and the panel would sit at doze 120Hz for
	 * good. This work re-arms the expiry independently of the kthread.
	 */
	struct delayed_work fod_hold_work;
  };

struct lhbm_setting {
	int lhbm_value;
	struct list_head link;
};

bool is_local_hbm(int disp_id);
bool mi_disp_lhbm_fod_enabled(struct dsi_panel *panel);
int mi_disp_lhbm_fod_thread_create(struct disp_feature *df, int disp_id);
int mi_disp_lhbm_fod_thread_destroy(struct disp_feature *df, int disp_id);
struct disp_lhbm_fod *mi_get_disp_lhbm_fod(int disp_id);
int mi_disp_lhbm_fod_allow_tx_lhbm(struct dsi_display *display, bool enable);
int mi_disp_lhbm_fod_update_layer_state(struct dsi_display *display,
		struct mi_layer_flags flags);
int mi_disp_lhbm_aod_to_normal_optimize(struct dsi_display *display,
		bool enable);
int mi_disp_set_local_hbm(int disp_id, int lhbm_value);
int mi_disp_lhbm_fod_watch_enable(struct disp_feature *df, int disp_id, bool enable);
int mi_disp_lhbm_fod_state_pub_get(int disp_id);
/*
 * keep_fod: true when this DPMS transition belongs to an in-flight FOD touch
 * (doze->normal walk), so the 3s doze-120Hz hold must survive it.
 */
void mi_disp_lhbm_fod_doze_nolp_abort(struct dsi_display *display,
		bool keep_fod);
bool mi_disp_lhbm_fod_nolp_active(struct dsi_panel *panel);
bool mi_disp_lhbm_fod_sdm_doze_active(struct dsi_display *display);
int mi_disp_update_0size_lhbm_layer(struct dsi_display *dsi_display,
			u32 mi_gxzw_flags);
int mi_disp_update_0size_lhbm_info(struct dsi_panel *panel);

#endif /* _MI_DISP_LHBM_H_ */

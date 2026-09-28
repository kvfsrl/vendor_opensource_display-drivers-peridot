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

#include "dsi_panel.h"
#include "dsi_display.h"
#include "mi_disp_feature.h"
#include "mi_sde_connector.h"

#define NEED_UPDATE_TO_FOD_FPS 120

/*
 * hoshikv FOD-HBM
 *   - doze 30Hz -> (FOD press) -> doze 120Hz + local HBM (HLPM)
 *   - release  -> HBM off, hold 120Hz for HOSHIKV_FOD_HOLD_MS
 *   - re-press -> reset hold timer (spam FOD tanpa bolak-balik 30/120)
 *   - timeout  -> back to doze 30Hz
 */
#define HOSHIKV_FOD_DEBOUNCE_N		2
#define HOSHIKV_FOD_HOLD_MS		3000
#define HOSHIKV_FOD_HOLD_REARM_MS	500
#define HOSHIKV_FOD_FPS_WAIT_MS		400
#define HOSHIKV_FOD_POLL_MS		20
#define HOSHIKV_FOD_TOUCH_MODE		10

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

	/* hoshikv FOD watch */
	struct task_struct *fod_watch_thread;
	wait_queue_head_t fod_watch_wq;
	atomic_t fod_watch_en;
	atomic_t fod_press;		/* debounced state */
	atomic_t fod_state_pub;		/* value exported to sysfs */
	int fod_raw_last;
	int fod_debounce;
	int fod_hold_armed;
	unsigned long fod_hold_deadline;
	unsigned long fod_fps_last_notify;
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
void mi_disp_lhbm_fod_watch_emit(int disp_id, int on);
int mi_disp_lhbm_fod_state_pub_get(int disp_id);
int mi_disp_update_0size_lhbm_layer(struct dsi_display *dsi_display,
			u32 mi_gxzw_flags);
int mi_disp_update_0size_lhbm_info(struct dsi_panel *panel);

#endif /* _MI_DISP_LHBM_H_ */

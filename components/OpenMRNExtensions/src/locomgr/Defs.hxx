/*
 * SPDX-FileCopyrightText: 2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 * 
 */

#ifndef LOCOMGR_DEFS_HXX_
#define LOCOMGR_DEFS_HXX_

#include <utils/constants.hxx>

/// Declares that Marklin locomotives are supported by the locomotive manager.
/// Defaults to TRUE.
DECLARE_CONST(trainmgr_support_marklin);

/// Declares that DCC locomotives are supported by the locomotive manager.
/// Defaults to TRUE.
DECLARE_CONST(trainmgr_support_dcc);

/// Declares that OpenLCB User addressed locomotives are supported by the
/// locomotive manager. Defaults to TRUE.
DECLARE_CONST(trainmgr_support_openlcb_user);

/// Declares that OpenLCB User addressed locomotives are supported by the
/// locomotive manager. Defaults to TRUE.
DECLARE_CONST(trainmgr_automatically_create_train_impl);

#endif // LOCOMGR_DEFS_HXX_
/*
 * SPDX-FileCopyrightText: 2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#include <utils/constants.hxx>

/// Default to enabling Marklin locomotives.
DEFAULT_CONST_TRUE(trainmgr_support_marklin);

/// Default to enabling DCC locomotives.
DEFAULT_CONST_TRUE(trainmgr_support_dcc);

/// Default to enabling OpenLCB User based locomotives.
DEFAULT_CONST_TRUE(trainmgr_support_openlcb_user);

/// Default to enabling automatic creation of locomotives in the database when
/// not found.
DEFAULT_CONST_TRUE(trainmgr_automatically_create_train_impl);

/*
 * SPDX-FileCopyrightText: 2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#include <utils/constants.hxx>

/// Number of milliseconds to delay before allocating a new train node.
DEFAULT_CONST(trainsearch_allocate_delay_ms, 200);

/// Number of milliseconds to delay between checks for new train node being
/// initialized checks.
DEFAULT_CONST(trainsearch_new_node_check_interval_ms, 1);

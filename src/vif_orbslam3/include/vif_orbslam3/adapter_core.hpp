// Copyright 2026 Mohit Singh
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

namespace vif_orbslam3
{

std::uint8_t ToVifTrackingState(int orb_state);

class RecoveryPulseDetector {
public:
  bool Observe(int orb_state);
  void Reset();

private:
  bool recovery_seen_{false};
};

}  // namespace vif_orbslam3

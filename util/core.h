#pragma once
#include <atomic>

namespace sailbot {

// Should be called at the start of every PROCESS (not Node).
void Init();
bool IsShutdown();
void RaiseShutdown();

}  // namespace sailboat

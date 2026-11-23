#pragma once
#include "Module.h"

// ---------------------------------------------------------------------
//  ModuleRegistry - a simple list of the modules that are switched on.
//  main.cpp adds modules to it; the registry starts them, updates them
//  at their own speed, and collects their status reports.
//  Modules run in the order they were added.
// ---------------------------------------------------------------------
class ModuleRegistry {
 public:
  bool add(Module& m) {
    if (_count >= kMax) return false;
    _mods[_count++] = &m;
    return true;
  }

  // Starts every module. Returns true only if all of them started.
  bool beginAll(Print& log) {
    bool allOk = true;
    for (size_t i = 0; i < _count; i++) {
      const bool ok = _mods[i]->begin();
      log.printf("[%s] %s\n", _mods[i]->name(), ok ? "ready" : "FAILED");
      allOk = allOk && ok;
    }
    return allOk;
  }

  void updateAll(uint32_t nowUs) {
    for (size_t i = 0; i < _count; i++) {
      if (_mods[i]->due(nowUs)) _mods[i]->update();
    }
  }

  void reportAll(Print& out) {
    for (size_t i = 0; i < _count; i++) _mods[i]->report(out);
  }

  size_t count() const { return _count; }

 private:
  static const size_t kMax = 16;
  Module* _mods[kMax];
  size_t _count = 0;
};

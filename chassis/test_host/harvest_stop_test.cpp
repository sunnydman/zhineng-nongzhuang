#include <assert.h>
#include "harvest_run.h"

constexpr HarvestStation station{100, 150, 350};

static void frame(HarvestRun& run, uint16_t seq, uint32_t ms, float mm, float speed,
                  char fruit = 'H', bool aligned = true, bool stopMode = true,
                  bool mayStrike = true) {
  HarvestFrame sight;
  sight.received = true;
  sight.seq = seq;
  sight.rxMs = ms;
  sight.nearFruit = true;
  sight.target = fruit;
  sight.aligned = aligned;
  sight.stopMode = stopMode;
  run.update(station, true, 1, mm, speed, mayStrike, sight, ms);
}

static HarvestRun approaching(char fruit = 'H') {
  HarvestRun run;
  frame(run, 1, 0, 0, 40, fruit);
  for (uint16_t seq = 2; seq < 5; ++seq) {
    frame(run, seq, seq * 100, 90, 40, fruit);
    assert(run.state() == HarvestRun::OUTSIDE);
  }
  for (uint16_t seq = 5; seq <= 8; ++seq) {
    frame(run, seq, seq * 100, 130, 40, fruit);
  }
  assert(run.state() == HarvestRun::SETTLE && run.holding());
  return run;
}

int main() {
  auto run = approaching('L');
  frame(run, 9, 900, 138, 0, 'L');
  frame(run, 10, 1000, 138, 12, 'L');
  frame(run, 11, 1100, 138, 0, 'L');
  frame(run, 11, 1130, 138, 0, 'L');
  assert(run.state() == HarvestRun::SETTLE);
  frame(run, 12, 1200, 138, 0, 'L');
  assert(run.state() == HarvestRun::STRIKE && run.target() == 'L' && run.holding());
  frame(run, 13, 2200, 138, 0, 'L');
  assert(run.state() == HarvestRun::DONE && !run.holding());
  run.advanceStation();
  frame(run, 14, 2300, 138, 0, 'L');
  assert(run.laneDistanceMm() == 138);

  run = approaching();
  for (uint16_t seq = 9; seq <= 21; ++seq) {
    frame(run, seq, seq * 100, 138, 0, 'L');
  }
  assert(run.state() == HarvestRun::DONE && !run.holding());

  run = approaching();
  frame(run, 9, 900, 138, 0, 'H', true, true, false);
  assert(run.state() == HarvestRun::DONE && !run.holding());

  run = approaching();
  frame(run, 9, 900, 351, 0);
  assert(run.state() == HarvestRun::DONE && !run.holding());

  run = HarvestRun{};
  frame(run, 1, 0, 0, 40);
  for (uint16_t seq = 2; seq < 9; ++seq) {
    frame(run, seq, seq * 100, 155, 40, 'H', true, false);
  }
  assert(run.state() == HarvestRun::SCAN && !run.holding());

  run = HarvestRun{};
  frame(run, 1, 0, 0, 40);
  for (uint16_t seq = 2; seq < 9; ++seq) {
    frame(run, seq, seq * 100, 155, 40, 'H', false);
  }
  assert(run.state() == HarvestRun::SCAN && !run.holding());
}

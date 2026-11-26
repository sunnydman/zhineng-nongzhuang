#include <assert.h>
#include <stdio.h>

#include "harvest_dual.h"

namespace {
HarvestFrame frame(uint16_t seq, uint32_t now, bool near, char target = 'N') {
  HarvestFrame result;
  result.received = true;
  result.seq = seq;
  result.rxMs = now;
  result.nearFruit = near;
  result.target = target;
  result.aligned = target != 'N';
  return result;
}
}

int main() {
  HarvestDual run;
  // 两边先见树。右侧只能在下一条仍对准的新帧发令。
  for (uint16_t seq = 1; seq <= 3; ++seq) {
    run.update(frame(seq, seq * 100, true, 'H'), frame(seq, seq * 100, true, 'L'),
               seq * 7, true, seq * 100);
  }
  assert(run.takeStrike(0) == 'H');
  assert(run.takeStrike(1) == 'N');
  run.update(frame(3, 300, true, 'H'), frame(3, 300, true, 'L'), 23, true, 380);
  assert(run.takeStrike(1) == 'N');
  run.update(frame(4, 400, true, 'H'), frame(4, 400, true, 'L'), 28, true, 400);
  assert(run.takeStrike(1) == 'L');
  assert(run.slow());

  // 左侧已经离树且走够距离，右侧全黄或空枝仍在画面，维持慢速。
  for (uint16_t seq = 5; seq <= 15; ++seq) {
    run.update(frame(seq, seq * 100, false), frame(seq, seq * 100, true),
               seq * 7, true, seq * 100);
    assert(run.takeStrike(0) == 'N' && run.takeStrike(1) == 'N');
    assert(run.slow());
  }
  assert(!run.occupied(0) && run.occupied(1));
  for (uint16_t seq = 16; seq <= 20; ++seq) {
    run.update(frame(seq, seq * 100, false), frame(seq, seq * 100, false),
               seq * 7, true, seq * 100);
    assert(run.slow());  // 空闲帧够了，但距刚才见树的位置还不够。
  }
  run.update(frame(21, 2100, false), frame(21, 2100, false), 160, true, 2100);
  assert(!run.slow());

  // 丢右路保持慢速。已清场的左侧遇到下一棵树时，速度/端头拦截能禁打。
  for (uint16_t seq = 22; seq <= 25; ++seq) {
    run.update(frame(seq, seq * 100, true, 'L'), frame(21, 2100, false),
               160 + 7 * (seq - 21), false, seq * 100);
    assert(run.takeStrike(0) == 'N');
    assert(run.slow());
  }
  run.update(frame(26, 2600, true, 'L'), frame(22, 2600, false), 200, false, 2600);
  assert(run.takeStrike(0) == 'N');
  run.update(frame(27, 2700, true, 'L'), frame(23, 2700, false), 210, true, 2700);
  assert(run.takeStrike(0) == 'L');
  run.update(frame(28, 2800, true, 'L'), frame(24, 2800, false), 220, true, 2800);
  assert(run.takeStrike(0) == 'N');

  HarvestDual lost;
  for (uint16_t seq = 1; seq <= 5; ++seq) {
    lost.update(frame(seq, seq * 100, false), {}, seq * 7, true, seq * 100);
  }
  assert(lost.slow());
  puts("PASS: staggered dual strike, inhibit gate, yellow/empty occupancy, duplicate and lost frames");
}

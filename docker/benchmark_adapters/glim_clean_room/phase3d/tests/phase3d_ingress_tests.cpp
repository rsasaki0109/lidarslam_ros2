#include "glim_clean_room/phase3d_ingress.hpp"
#include "glim_clean_room/phase3d_output_contract.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace {

using glim_clean_room::ErrorCode;
using glim_clean_room::StampNanoseconds;
using glim_clean_room::phase3d::BoundedIngressQueue;
using glim_clean_room::phase3d::IngressItem;
using glim_clean_room::phase3d::IngressKind;
using glim_clean_room::phase3d::checked_row_step;

IngressItem item(StampNanoseconds stamp, std::uint64_t arrival,
                 IngressKind kind = IngressKind::kLidar) {
  return IngressItem{stamp, arrival, kind};
}

TEST(Phase3dIngress, EqualStampsAreOrderedByHostArrivalOrdinal) {
  BoundedIngressQueue queue(4, 0);
  ASSERT_TRUE(queue.enqueue(item(10, 0)));
  ASSERT_TRUE(queue.enqueue(item(10, 1, IngressKind::kImu)));
  const auto flushed = queue.flush_ready(false);
  ASSERT_TRUE(flushed);
  ASSERT_EQ(flushed.value.size(), 2U);
  EXPECT_EQ(flushed.value[0].arrival, 0U);
  EXPECT_EQ(flushed.value[1].arrival, 1U);
}

TEST(Phase3dIngress, SignedInt64CutoffDoesNotWrap) {
  BoundedIngressQueue queue(4, 3);
  ASSERT_TRUE(queue.enqueue(item(std::numeric_limits<StampNanoseconds>::min(), 0)));
  ASSERT_TRUE(queue.enqueue(item(std::numeric_limits<StampNanoseconds>::min() + 2, 1)));
  const auto early = queue.flush_ready(false);
  ASSERT_TRUE(early);
  ASSERT_EQ(early.value.size(), 1U);
  EXPECT_EQ(early.value.front().stamp,
            std::numeric_limits<StampNanoseconds>::min());

  BoundedIngressQueue high(4, 3);
  ASSERT_TRUE(high.enqueue(item(std::numeric_limits<StampNanoseconds>::max() - 4, 0)));
  ASSERT_TRUE(high.enqueue(item(std::numeric_limits<StampNanoseconds>::max(), 1)));
  const auto late = high.flush_ready(false);
  ASSERT_TRUE(late);
  ASSERT_EQ(late.value.size(), 1U);
  EXPECT_EQ(late.value.front().stamp,
            std::numeric_limits<StampNanoseconds>::max() - 4);
}

TEST(Phase3dIngress, ReorderWindowFlushesOnlyReadyItems) {
  BoundedIngressQueue queue(4, 10);
  ASSERT_TRUE(queue.enqueue(item(100, 0)));
  ASSERT_TRUE(queue.enqueue(item(105, 1)));
  auto flushed = queue.flush_ready(false);
  ASSERT_TRUE(flushed);
  EXPECT_TRUE(flushed.value.empty());
  ASSERT_TRUE(queue.enqueue(item(120, 2)));
  flushed = queue.flush_ready(false);
  ASSERT_TRUE(flushed);
  ASSERT_EQ(flushed.value.size(), 2U);
  EXPECT_EQ(flushed.value[0].stamp, 100);
  EXPECT_EQ(flushed.value[1].stamp, 105);
}

TEST(Phase3dIngress, CapacityFailureDropsNothingSilentlyAndLatches) {
  BoundedIngressQueue queue(1, 0);
  ASSERT_TRUE(queue.enqueue(item(1, 0)));
  const auto rejected = queue.enqueue(item(2, 1));
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error.code, ErrorCode::kLedgerOverflow);
  EXPECT_TRUE(queue.terminal());
  EXPECT_EQ(queue.pending_size(), 1U);
  EXPECT_EQ(queue.counters().enqueued, 1U);
  EXPECT_EQ(queue.counters().rejected, 1U);
}

TEST(Phase3dIngress, PostEofAndDoubleEofAreTerminal) {
  BoundedIngressQueue queue(4, 0);
  ASSERT_TRUE(queue.enqueue(item(1, 0)));
  ASSERT_TRUE(queue.request_eof());
  const auto post_eof = queue.enqueue(item(2, 1));
  ASSERT_FALSE(post_eof);
  EXPECT_EQ(post_eof.error.code, ErrorCode::kPostEof);
  EXPECT_TRUE(queue.terminal());

  BoundedIngressQueue second(4, 0);
  ASSERT_TRUE(second.request_eof());
  const auto duplicate = second.request_eof();
  ASSERT_FALSE(duplicate);
  EXPECT_EQ(duplicate.error.code, ErrorCode::kInvalidState);
}

TEST(Phase3dIngress, SubmittedStampRegressionIsRejected) {
  BoundedIngressQueue queue(4, 0);
  ASSERT_TRUE(queue.note_submitted(100));
  const auto status = queue.note_submitted(99);
  ASSERT_FALSE(status);
  EXPECT_EQ(status.error.code, ErrorCode::kNonMonotonicStamp);
  EXPECT_TRUE(queue.terminal());
}

TEST(Phase3dIngress, EofFlushIsExplicitAndDrainsAll) {
  BoundedIngressQueue invalid(4, 100);
  ASSERT_TRUE(invalid.enqueue(item(10, 0)));
  EXPECT_FALSE(invalid.flush_ready(true));

  BoundedIngressQueue queue(4, 100);
  ASSERT_TRUE(queue.enqueue(item(10, 0)));
  ASSERT_TRUE(queue.enqueue(item(1, 1)));
  ASSERT_TRUE(queue.request_eof());
  const auto drained = queue.flush_ready(true);
  ASSERT_TRUE(drained);
  ASSERT_EQ(drained.value.size(), 2U);
  EXPECT_EQ(drained.value[0].stamp, 1);
  EXPECT_EQ(drained.value[1].stamp, 10);
  EXPECT_EQ(queue.pending_size(), 0U);
}

TEST(Phase3dOutputContract, RowStepOverflowFailsBeforeAllocation) {
  const auto overflow = checked_row_step(std::numeric_limits<std::uint32_t>::max(), 24);
  ASSERT_FALSE(overflow);
  EXPECT_EQ(overflow.error.code, ErrorCode::kLedgerOverflow);
  const auto exact = checked_row_step(
      std::numeric_limits<std::uint32_t>::max() / 24, 24);
  ASSERT_TRUE(exact);
  EXPECT_EQ(exact.value,
            (std::numeric_limits<std::uint32_t>::max() / 24) * 24U);
}

TEST(Phase3dOutputContract, RosStampAndTrajectoryShapeAreFailClosed) {
  const auto negative = glim_clean_room::event_stamp_from_ros(0, -1, 0);
  ASSERT_TRUE(negative);
  EXPECT_EQ(negative.value.nanoseconds, -1000000000LL);
  const auto max_valid = glim_clean_room::event_stamp_from_ros(
      0, 9223372036LL, 854775807LL);
  ASSERT_TRUE(max_valid);
  const auto overflow = glim_clean_room::event_stamp_from_ros(
      0, 9223372036LL, 854775808LL);
  ASSERT_FALSE(overflow);
  EXPECT_EQ(overflow.error.code, ErrorCode::kInvalidStamp);

  glim_clean_room::TrajectorySample sample;
  sample.frame_id = "world";
  sample.pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  EXPECT_TRUE(glim_clean_room::validate_trajectory_sample(sample));
  sample.pose[6] = 2.0;
  EXPECT_EQ(glim_clean_room::validate_trajectory_sample(sample).error.code,
            ErrorCode::kInvalidLayout);
  sample.pose[6] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(glim_clean_room::validate_trajectory_sample(sample).error.code,
            ErrorCode::kNonFinite);
}

}  // namespace

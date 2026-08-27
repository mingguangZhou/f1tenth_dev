#include <limits>

#include "gtest/gtest.h"

#include "path_following_v2/candidate_selection.hpp"

namespace selection = path_following_v2::selection;

TEST(CandidateSelection, RejectsTwoInvalidCandidates)
{
  EXPECT_EQ(
    selection::chooseSaferCandidate({}, {}, 0.03),
    selection::Choice::NONE);
}

TEST(CandidateSelection, ChoosesTheOnlyValidCandidate)
{
  selection::Metrics left;
  selection::Metrics right;
  right.valid = true;

  EXPECT_EQ(
    selection::chooseSaferCandidate(left, right, 0.03),
    selection::Choice::RIGHT);
}

TEST(CandidateSelection, MateriallyWiderPassageBeatsLowerObjectiveCost)
{
  selection::Metrics left;
  left.valid = true;
  left.minimum_clearance = 0.31;
  left.objective_cost = 1.0;

  selection::Metrics right;
  right.valid = true;
  right.minimum_clearance = 0.55;
  right.objective_cost = 20.0;

  EXPECT_EQ(
    selection::chooseSaferCandidate(left, right, 0.03),
    selection::Choice::RIGHT);
}

TEST(CandidateSelection, ObjectiveBreaksAComparableClearanceTie)
{
  selection::Metrics left;
  left.valid = true;
  left.minimum_clearance = 0.40;
  left.objective_cost = 2.0;
  left.objective_domain = selection::ObjectiveDomain::LATTICE;

  selection::Metrics right;
  right.valid = true;
  right.minimum_clearance = 0.42;
  right.objective_cost = 3.0;
  right.objective_domain = selection::ObjectiveDomain::LATTICE;

  EXPECT_EQ(
    selection::chooseSaferCandidate(left, right, 0.03),
    selection::Choice::LEFT);
}

TEST(CandidateSelection, DoesNotCompareCostsFromDifferentPlanningBackends)
{
  selection::Metrics left;
  left.valid = true;
  left.minimum_clearance = 0.30;
  left.objective_cost = std::numeric_limits<double>::infinity();
  left.maximum_curvature = 0.50;
  left.peak_offset = 0.15;
  left.objective_domain = selection::ObjectiveDomain::NONE;

  selection::Metrics right;
  right.valid = true;
  right.minimum_clearance = 0.31;
  right.objective_cost = 1.0;
  right.maximum_curvature = 0.80;
  right.peak_offset = -0.80;
  right.objective_domain = selection::ObjectiveDomain::LATTICE;

  EXPECT_EQ(
    selection::chooseSaferCandidate(left, right, 0.03),
    selection::Choice::LEFT);
}

TEST(CandidateSelection, UnboundedPassageBeatsFiniteBottleneck)
{
  selection::Metrics left;
  left.valid = true;
  left.minimum_clearance = std::numeric_limits<double>::infinity();
  left.objective_cost = 10.0;

  selection::Metrics right;
  right.valid = true;
  right.minimum_clearance = 0.50;
  right.objective_cost = 1.0;

  EXPECT_EQ(
    selection::chooseSaferCandidate(left, right, 0.03),
    selection::Choice::LEFT);
}

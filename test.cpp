#include <list>
#include <memory>
#include <algorithm>

#include "gtest/gtest.h"

#include "point-selector.h"

struct Point {
  Point(float in_x, float in_y) : x(in_x), y(in_y), selected(false) {}
  float x;
  float y;
  bool selected;
};

class PointHandle {
 public:
  std::weak_ptr<Point> v_;

  PointHandle() = default;
  PointHandle(std::weak_ptr<Point> in_v) : v_(in_v) {}

  ptsel::Vector2 getLocation() const {
    if (isValid()) {
      auto vec = v_.lock();
      return {vec->x, vec->y};
    } else {
      return {0.f, 0.f};
    }
  }

  bool isValid() const { return !v_.expired(); }

  friend bool operator==(const PointHandle& lhs, const PointHandle& rhs);

  void OnSelected() {
    if (isValid()) {
      v_.lock()->selected = true;
    }
  }

  void OnDeselected() {
    if (isValid()) {
      v_.lock()->selected = false;
    }
  }
};

bool operator==(const PointHandle& lhs, const PointHandle& rhs) {
  if (!(lhs.v_.expired() || rhs.v_.expired())) {
    auto locked_v = lhs.v_.lock();
    auto locked_other_v = rhs.v_.lock();
    return locked_v == locked_other_v;
  }
  return false;
}

class PointSelectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    points_ = {std::make_shared<Point>(-4.f, 3.f),
               std::make_shared<Point>(0.f, 3.5f),
               std::make_shared<Point>(-2.f, -2.f),
               std::make_shared<Point>(4.f, 0.f),
               std::make_shared<Point>(3.f, -3.f),
               std::make_shared<Point>(3.5f, 4.f)};
    for (auto&& p : points_) {
      point_handles_.push_back(PointHandle(p));
    }

    ps_ = ptsel::PointSelector<PointHandle>(point_handles_);
  }
  std::list<std::shared_ptr<Point>> points_;
  std::vector<PointHandle> point_handles_;
  ptsel::PointSelector<PointHandle> ps_;
};

TEST_F(PointSelectorTest, SelectingLocationsWithPointsSelectsThosePoints) {
  PointHandle selectedPoint = ps_.selectLocation({0.f, 3.5f});

  EXPECT_TRUE(selectedPoint == point_handles_[1]);

  selectedPoint = ps_.selectLocation({3.f, -3.f});

  EXPECT_TRUE(selectedPoint == point_handles_[4]);
  EXPECT_TRUE(ps_.getSelectedPoints().size() == 2);
}

TEST_F(PointSelectorTest, SelectingLocationsWithoutPointsSelectsNothing) {
  PointHandle selectedPoint = ps_.selectLocation({0.f, 1.f});

  EXPECT_FALSE(selectedPoint.isValid());

  ps_.selectLocation({-2.f, -2.f});  // selecting valid location
  selectedPoint = ps_.selectLocation({-3.f, 1.f});

  EXPECT_FALSE(selectedPoint.isValid());
  EXPECT_TRUE(ps_.getSelectedPoints().size() == 1);
}

TEST_F(PointSelectorTest, ClearingSelectionWorks) {
  EXPECT_TRUE(ps_.getSelectedPoints().size() == 0);

  ps_.selectLocation({0.f, 3.5f});
  ps_.selectLocation({3.f, -3.f});

  EXPECT_TRUE(ps_.getSelectedPoints().size() == 2);

  ps_.clearSelection();

  EXPECT_TRUE(ps_.getSelectedPoints().size() == 0);
}

TEST_F(PointSelectorTest, SelectedPointsAreSortedClockwise) {
  ps_.selectLocation({4.f, 0.f});
  ps_.selectLocation({-2.f, -2.f});
  ps_.selectLocation({-4.f, 3.f});
  auto selected = ps_.getSelectedPoints();

  EXPECT_TRUE(selected.size() == 3 && selected[0] == point_handles_[0] &&
              selected[1] == point_handles_[3] &&
              selected[2] == point_handles_[2]);

  ps_.clearSelection();

  ps_.selectLocation({3.5f, 4.f});
  ps_.selectLocation({4.f, 0.f});
  ps_.selectLocation({0.f, 3.5f});
  ps_.selectLocation({3.f, -3.f});
  selected = ps_.getSelectedPoints();

  EXPECT_TRUE(selected.size() == 4 && selected[0] == point_handles_[1] &&
              selected[1] == point_handles_[5] &&
              selected[2] == point_handles_[3] &&
              selected[3] == point_handles_[4]);
}

TEST_F(PointSelectorTest, SelectingBoxAreaSelectsPointsWithinBox) {
  std::vector<PointHandle> selectedPoints =
      ps_.selectArea({{1.f, 1.f}, {5.f, -1.f}});

  EXPECT_TRUE(selectedPoints.size() == 1 &&
              selectedPoints[0] == point_handles_[3]);

  selectedPoints = ps_.selectArea({{1.f, 2.f}, {-4.f, 4.f}});
  bool firstPointSelected =
      std::find(selectedPoints.begin(), selectedPoints.end(),
                point_handles_[0]) != selectedPoints.end();
  bool secondPointSelected =
      std::find(selectedPoints.begin(), selectedPoints.end(),
                point_handles_[1]) != selectedPoints.end();

  EXPECT_TRUE(selectedPoints.size() == 2 && firstPointSelected &&
              secondPointSelected);
  EXPECT_TRUE(ps_.getSelectedPoints().size() == 3);
}

TEST_F(PointSelectorTest, SelectingBoxAreaReturnsOnlyNewlySelectedPoints) {
  ps_.selectArea({{-1.f, 5.f}, {5.f, -1.f}});
  // selecting same area again
  auto selectedPoints = ps_.selectArea({{-1.f, 5.f}, {5.f, -1.f}});

  EXPECT_TRUE(selectedPoints.empty());

  selectedPoints = ps_.selectArea({{5.f, 5.f}, {-6.f, -1.f}});

  EXPECT_TRUE(selectedPoints.size() == 1 &&
              selectedPoints[0] == point_handles_[0]);
  EXPECT_TRUE(ps_.getSelectedPoints().size() == 4);
}

TEST_F(PointSelectorTest, SettingPointsAllowsNewPointsToBeSelected) {
  points_.push_back(std::make_shared<Point>(2.f, 2.f));

  ps_.selectLocation({2.f, 2.f});
  EXPECT_TRUE(ps_.getSelectedPoints().size() == 0);

  point_handles_.clear();
  for (auto&& p : points_) {
    point_handles_.push_back(PointHandle(p));
  }

  ps_.setPoints(point_handles_);

  ps_.selectLocation({2.f, 2.f});
  EXPECT_TRUE(ps_.getSelectedPoints().size() == 1);
}

TEST_F(PointSelectorTest,
       SettingPointsRemovesFromSelectionPointsThatAreNotInNewPointSet) {
  ps_.selectArea({{-3.f, 2.5f}, {5.f, -4.f}});
  EXPECT_TRUE(ps_.getSelectedPoints().size() == 3);

  auto removedPointIter = points_.end();
  removedPointIter--;
  removedPointIter--;
  auto pointToRemove = *removedPointIter;
  points_.remove(pointToRemove);

  point_handles_.clear();
  for (auto&& p : points_) {
    point_handles_.push_back(PointHandle(p));
  }

  ps_.setPoints(point_handles_);
  auto selectedPoints = ps_.getSelectedPoints();
  bool pointRemoved =
      std::find(selectedPoints.begin(), selectedPoints.end(),
                PointHandle(pointToRemove)) == selectedPoints.end();
  EXPECT_TRUE(selectedPoints.size() == 2);
  EXPECT_TRUE(pointRemoved);
}

TEST_F(PointSelectorTest, SelectingPointsCallsOnSelected) {
  ps_.selectLocation({0.f, 3.5f});

  auto selectedPointIter = points_.begin();
  ++selectedPointIter;

  EXPECT_TRUE((*selectedPointIter)->selected == true);

  ps_.selectArea({{-1.f, 5.f}, {5.f, -1.f}});

  selectedPointIter = points_.end();
  --selectedPointIter;
  EXPECT_TRUE((*selectedPointIter)->selected == true);
  --selectedPointIter;
  --selectedPointIter;
  EXPECT_TRUE((*selectedPointIter)->selected == true);
}

TEST_F(PointSelectorTest, ClearingSelectionCallsOnDeselected) {
  ps_.selectLocation({0.f, 3.5f});
  ps_.selectArea({{-1.f, 5.f}, {5.f, -1.f}});
  ps_.clearSelection();

  auto selectedPointIter = points_.begin();
  ++selectedPointIter;
  EXPECT_TRUE((*selectedPointIter)->selected == false);
  ++selectedPointIter;
  ++selectedPointIter;
  EXPECT_TRUE((*selectedPointIter)->selected == false);
  ++selectedPointIter;
  ++selectedPointIter;
  EXPECT_TRUE((*selectedPointIter)->selected == false);
}
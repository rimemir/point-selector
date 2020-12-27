#ifndef POINT_SELECTOR_H
#define POINT_SELECTOR_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace ptsel {

struct Vector2 {
  float x;
  float y;
};

inline Vector2 operator+(Vector2 v1, Vector2 v2) {
  return {v1.x + v2.x, v1.y + v2.y};
}

inline Vector2 operator-(Vector2 v) { return {-v.x, -v.y}; }

inline Vector2 operator-(Vector2 v1, Vector2 v2) { return v1 + (-v2); }

inline float lengthSquared(Vector2 v) { return v.x * v.x + v.y * v.y; }

inline float distanceSquared(Vector2 v1, Vector2 v2) {
  return lengthSquared(v1 - v2);
}

inline Vector2 operator/(Vector2 v, float s) { return {v.x / s, v.y / s}; }

inline Vector2 centroid(const std::vector<Vector2>& vectors) {
  return std::accumulate(vectors.begin(), vectors.end(), Vector2{0.f, 0.f},
                         [](Vector2 v1, Vector2 v2) { return v1 + v2; }) /
         (float)vectors.size();
}

class Box2 {
 public:
  Box2(Vector2 start, Vector2 end) {
    bottomLeft_.x = std::min(start.x, end.x);
    bottomLeft_.y = std::min(start.y, end.y);
    topRight_.x = std::max(start.x, end.x);
    topRight_.y = std::max(start.y, end.y);
  }

  bool contains(Vector2 l) const {
    return l.x >= bottomLeft_.x && l.x <= topRight_.x && l.y >= bottomLeft_.y &&
           l.y <= topRight_.y;
  }

 private:
  Vector2 bottomLeft_;
  Vector2 topRight_;
};

/**
 * PointSelector takes a vector of PointHandleT objects, and allows
 * selection/filtering of those handles with calls to selectLocation() and
 * selectArea(). Selected points can be accessed at any time by a call to
 * getSelectedPoints().
 *
 * Type template argument PointHandleT represents a handle/reference to an
 * object representing a 2D point. Requirements for this type are:
 *  - getLocation() function that returns a Vector2 representing the coordinates
 *    of the point
 *  - isValid() function that returns a bool value telling if the handle refers
 *    to a valid/live point
 *  - operator== which returns true if two compared handles refer to the same
 *    point object
 *  - OnSelected() function that gets called when the point is selected
 *  - OnDeselected() function that gets called when the point is deselected
 *  - default constructor
 */
template <typename PointHandleT>
class PointSelector {
 public:
  PointSelector() = default;
  PointSelector(const std::vector<PointHandleT>& points) : points_(points) {}

 public:
  /**
   * Takes a 2D location and looks for a closest point in a radius (set by
   * setSelectionRadius()) around that location, adding it to selection. Returns
   * a handle to the point if it found one and it was added to selection (was
   * not previously selected), or invalid handle otherwise.
   */
  PointHandleT selectLocation(Vector2 l) {
    std::vector<PointHandleT> pointsInRadius =
        getPointsInRadius(points_, l, selectionRadius_);
    if (!pointsInRadius.empty()) {
      PointHandleT closestPoint = getClosestPoint(pointsInRadius, l);
      std::vector<PointHandleT> added = addToSelection({closestPoint});
      if (added.size() == 1) {
        return closestPoint;
      }
    }
    return PointHandleT();
  }

  /**
   * Takes a 2D box (rectangular area) and looks for points inside the box,
   * adding all found points to selection. Returns a vector of points that were
   * added to selection (those that were not already selected), or empty vector
   * if nothing new was selected.
   */
  std::vector<PointHandleT> selectArea(const Box2& b) {
    std::vector<PointHandleT> pointsInBox = getPointsInBox(points_, b);
    if (!pointsInBox.empty()) {
      std::vector<PointHandleT> selectedPoints = addToSelection(pointsInBox);
      return selectedPoints;
    }
    return {};
  }

  /**
   * Returns a vector of currently selected points, sorted based on the angle
   * they make with the negative x axis of a coordinate system centered on the
   * centroid of all selected points.
   */
  std::vector<PointHandleT> getSelectedPoints() const {
    if (sortIsDirty_) {
      sortSelected();
    }
    return selectedPoints_;
  }

  /**
   * Deselects all selected points. getSelectedPoints() returns an empty vector
   * after this call.
   */
  void clearSelection() {
    auto removedPoints = selectedPoints_;
    selectedPoints_.clear();
    for (auto removedPoint : removedPoints) {
      removedPoint.OnDeselected();
    }
  }

  /**
   * Sets the vector of points being selected/filtered. Any points that are
   * already selected, and not in the new vector, are removed from selection
   * (from vector returned by getSelectedPoints()).
   */
  void setPoints(const std::vector<PointHandleT>& points) {
    points_ = points;
    auto newEnd = std::remove_if(selectedPoints_.begin(), selectedPoints_.end(),
                                 [this](const PointHandleT& p) {
                                   if (!p.isValid()) {
                                     return true;
                                   }
                                   bool notInNewPoints =
                                       std::find(points_.begin(), points_.end(),
                                                 p) == points_.end();
                                   return notInNewPoints;
                                 });
    selectedPoints_.erase(newEnd, selectedPoints_.end());
  }

  /**
   * Sets the search radius used when selecting individual locations with
   * selectLocation().
   */
  void setSelectionRadius(float r) { selectionRadius_ = r; }

 private:
  std::vector<PointHandleT> addToSelection(
      const std::vector<PointHandleT> points) {
    std::vector<PointHandleT> pointsToAdd;
    std::copy_if(points.begin(), points.end(), std::back_inserter(pointsToAdd),
                 [this](const PointHandleT& p) {
                   bool notInSelected =
                       std::find(selectedPoints_.begin(), selectedPoints_.end(),
                                 p) == selectedPoints_.end();
                   return notInSelected;
                 });

    std::copy(pointsToAdd.begin(), pointsToAdd.end(),
              std::back_inserter(selectedPoints_));

    for (auto addedPoint : pointsToAdd) {
      addedPoint.OnSelected();
    }

    sortIsDirty_ = !pointsToAdd.empty();

    return pointsToAdd;
  }

  void sortSelected() const {
    Vector2 centroid = getCentroid(selectedPoints_);
    std::sort(selectedPoints_.begin(), selectedPoints_.end(),
              [centroid](const PointHandleT& p1, const PointHandleT& p2) {
                if (p1.isValid() && !p2.isValid()) {
                  return true;
                } else if (!(p1.isValid() || p2.isValid())) {
                  return false;
                }
                Vector2 d1 = p1.getLocation() - centroid;
                Vector2 d2 = p2.getLocation() - centroid;
                return std::atan2f(d1.y, d1.x) > std::atan2f(d2.y, d2.x);
              });
    sortIsDirty_ = false;
  }

  Vector2 getCentroid(const std::vector<PointHandleT>& points) const {
    return centroid(getPointLocations(points));
  }

  std::vector<Vector2> getPointLocations(
      const std::vector<PointHandleT>& points) const {
    std::vector<Vector2> locations;
    std::for_each(points.begin(), points.end(),
                  [&locations](const PointHandleT& p) {
                    if (p.isValid()) {
                      locations.push_back(p.getLocation());
                    }
                  });
    return locations;
  }

  PointHandleT getClosestPoint(const std::vector<PointHandleT>& points,
                               Vector2 l) const {
    if (points.empty()) {
      return PointHandleT();
    }
    return *std::min_element(
        points.begin(), points.end(),
        [l](const PointHandleT& p1, const PointHandleT& p2) {
          return distanceSquared(p1.getLocation(), l) <
                 distanceSquared(p2.getLocation(), l);
        });
  }

  std::vector<PointHandleT> getPointsInRadius(
      const std::vector<PointHandleT>& points, Vector2 l, float r) const {
    std::vector<PointHandleT> pointsInRadius;
    std::copy_if(
        points.begin(), points.end(), std::back_inserter(pointsInRadius),
        [l, r](const PointHandleT& p) {
          return p.isValid() && distanceSquared(p.getLocation(), l) < r * r;
        });
    return pointsInRadius;
  }

  std::vector<PointHandleT> getPointsInBox(
      const std::vector<PointHandleT>& points, const Box2& box) const {
    std::vector<PointHandleT> pointsInBox;
    std::copy_if(points.begin(), points.end(), std::back_inserter(pointsInBox),
                 [box](const PointHandleT& p) {
                   return p.isValid() && box.contains(p.getLocation());
                 });
    return pointsInBox;
  }

 private:
  std::vector<PointHandleT> points_;
  mutable std::vector<PointHandleT> selectedPoints_;

  mutable bool sortIsDirty_;

  float selectionRadius_ = 0.1f;
};
}  // namespace ptsel

#endif
# Introduction

This is a simple header-only library for selecting (or filtering) a set of 2D points. It allows 
selecting points by sending individual locations, or selecting all points in a box.

# Usage

Main class is 
```C++ 
template<typename PointHandleT> 
class PointSelector
```
An instance of this class manages selection of a set of points that are sent to it. Points are sent to an instance of 
`PointSelector` by passing a vector of objects of a custom `PointHandleT` type, either in the constructor
```C++
std::vector<MyPointHandle> points; /* filled somehow */
ptsel::PointSelector<MyPointHandle> ps(points);
```
 or by calling `setPoints` afterwards.
```C++
ptsel::PointSelector<MyPointHandle> ps;
std::vector<MyPointHandle> points; /* filled somehow */
ps.setPoints(points);
```

Template parameter `PointHandleT` is the main interface between user code and the library. It represents a handle or reference
to user's point objects (which can be anything that has a 2D location). `PointSelector` expects the `PointHandleT` type to have:
- `ptsel::Vector2 getLocation() const` gives the coordinates of the point refered to by the handle
- `bool isValid() const` tells if the handle still refers to a valid point.
- `operator==` which should return true when the two compared handles refer to the same point object
- `OnSelected()` function that gets called whenever the point is selected
- `OnDeselected()` function that gets called whenever the point is deselected
- a default constructor

After creating an instance of `PointSelector`, points are selected using
```C++
PointHandleT selectLocation(Vector2 l)
```
or
```C++
std::vector<PointHandleT> selectArea(const Box2& b)
```
Currently selected points can be retrieved at any time using
```C++
std::vector<PointHandleT> getSelectedPoints() const
```

`selectLocation` takes a 2D location and searches for a point at that location, with a tolerance radius that can be set with `setSelectionRadius`.
If it finds a point, and it is not already selected, it adds it to selection and returns a handle to that point, otherwise it returns an 
invalid handle. If it finds multiple points within the tolerance radius, it picks the closest one to the location.

`selectArea` takes an axis-aligned 2D box that is constructed from two 2D locations. It searches for points that are within
the box, and adds to selection all that are not already selected. It returns a vector of handles to all
points that are added to selection.

Selection can be cleared with
```C++
void clearSelection()
```
which deselects all points, and after which `getSelectedPoints` returns an empty vector until new points
are selected.

# Building tests

[CMake](https://cmake.org/) can be used to build and run the tests. Testing library used is [googletest](https://github.com/google/googletest).
googletest should be installed with CMake so that `find_package` can find it.

# TODO

- Optimise the point searching, adding and removing, probably by using a data structure like [quadtree](https://en.wikipedia.org/wiki/Quadtree)
or [k-d tree](https://en.wikipedia.org/wiki/K-d_tree).
- Allow deselection of individual points or points in the area, symmetric to selecting
- Add a way to distinguish between selecting an already selected point/s and selecting an empty location
- Make OnSelected() and OnDeselected() callbacks optional
- ...

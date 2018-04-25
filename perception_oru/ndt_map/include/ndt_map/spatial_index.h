#ifndef SPATIAL_INDEX_HH
#define SPATIAL_INDEX_HH

#include <vector>
#include <iostream>

#include <ndt_map/ndt_cell.h>

namespace lslgeneric {

/** \brief Base class for all spatial indexing structures
    \details
A SpatialIndex is anything that holds PointInterface pointers
and organizes them in a manner accessible from outside.
This class defines what is necessary to be a spatial index - namely
the ability to find the cell in which a point is placed and to store
newly observed points. It should also be possible to check the size of
the occupied space, as well as to get cells neighboring any given cell.
*/
class SpatialIndex {
protected:

public:
    typedef std::vector<NDTCell*> CellPtrVector;
    typedef typename CellPtrVector::iterator CellVectorItr;
    typedef typename CellPtrVector::const_iterator CellVectorConstItr;

    virtual ~SpatialIndex() {
    }

    virtual NDTCell* getCellForPoint(const pcl::PointXYZ &point) = 0;
    ///add a point and get back the pointer to the cell in which it ended up
    virtual NDTCell* addPoint(const pcl::PointXYZ &point) = 0;

    ///iterator through all cells in index, points at the begining
    virtual CellVectorItr begin() = 0;
    virtual CellVectorConstItr begin() const = 0;
    ///iterator through all cells in index, points at the end
    virtual CellVectorItr end() = 0;
    virtual CellVectorConstItr end() const = 0;

    // should be 'pure'?
    virtual int size() const {
        return -1;
    }

    ///clone - create an empty object with same type
    virtual SpatialIndex* clone() const = 0;
    ///copy - create the same object as a new instance
    virtual SpatialIndex* copy() const = 0;

    ///the following methods provide index specific functionalities and
    ///don't have to be implemented by all sub-classes
    virtual void setCenter(const double &cx, const double &cy, const double &cz) {}
    virtual void setSize(const double &sx, const double &sy, const double &sz) {}

    ///method to return all cells within a certain radius from a point
    virtual void getNeighbors(const pcl::PointXYZ &point, const double &radius, std::vector<NDTCell*> &cells) = 0;

    ///sets the cell factory type
    virtual void setCellType(NDTCell *type) = 0;

    ///reads map contents from .jff file
    virtual int loadFromJFF(FILE * jffin) const {
        std::cerr << "Calling from SpatialIndex.h\n";
        return -1;
    }
};

} //end namespace


#endif

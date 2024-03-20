

#pragma once

#include <pcl/filters/boost.h>
#include <pcl/filters/filter.h>
#include <map>

namespace pcl
{

    PCL_EXPORTS void
    getMinMax3D(const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt);

    PCL_EXPORTS void
    getMinMax3D(const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                const std::string &distance_field_name, float min_distance, float max_distance,
                Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative);

    inline Eigen::MatrixXi
    getHalfNeighborCellIndices();

    inline Eigen::MatrixXi
    getAllNeighborCellIndices();

    template<typename PointT>
    void
    getMinMax3D(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                const std::string &distance_field_name, float min_distance, float max_distance,
                Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative);

    template<typename PointT>
    void
    getMinMax3D(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                const std::vector<int> &indices,
                const std::string &distance_field_name, float min_distance, float max_distance,
                Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative);

    template<typename PointT>
    class VoxelGridLarge : public Filter<PointT>
    {
    protected:
        using Filter<PointT>::filter_name_;
        using Filter<PointT>::getClassName;
        using Filter<PointT>::input_;
        using Filter<PointT>::indices_;

        typedef typename Filter<PointT>::PointCloud PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

        typedef boost::shared_ptr<VoxelGridLarge<PointT> > Ptr;
        typedef boost::shared_ptr<const VoxelGridLarge<PointT> > ConstPtr;

        VoxelGridLarge() :
                leaf_size_(Eigen::Vector4f::Zero()),
                inverse_leaf_size_(Eigen::Array4f::Zero()),
                downsample_all_data_(true),
                save_leaf_layout_(false),
                min_b_(Eigen::Vector4i::Zero()),
                max_b_(Eigen::Vector4i::Zero()),
                div_b_(Eigen::Vector4i::Zero()),
                divb_mul_(Eigen::Vector4i::Zero()),
                filter_field_name_(""),
                filter_limit_min_(-FLT_MAX),
                filter_limit_max_(FLT_MAX),
                filter_limit_negative_(false),
                min_points_per_voxel_(0)
        {
            filter_name_ = "VoxelGridLarge";
        }

        ~VoxelGridLarge()
        {
        }

        inline void
        setLeafSize(const Eigen::Vector4f &leaf_size)
        {
            leaf_size_ = leaf_size;

            if (leaf_size_[3] == 0)
                leaf_size_[3] = 1;

            inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
        }

        inline void
        setLeafSize(float lx, float ly, float lz)
        {
            leaf_size_[0] = lx;
            leaf_size_[1] = ly;
            leaf_size_[2] = lz;

            if (leaf_size_[3] == 0)
                leaf_size_[3] = 1;

            inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
        }

        inline Eigen::Vector3f
        getLeafSize() const { return (leaf_size_.head<3>()); }

        inline void
        setDownsampleAllData(bool downsample) { downsample_all_data_ = downsample; }

        inline bool
        getDownsampleAllData() const { return (downsample_all_data_); }

        inline void
        setMinimumPointsNumberPerVoxel(unsigned int min_points_per_voxel) { min_points_per_voxel_ = min_points_per_voxel; }

        inline unsigned int
        getMinimumPointsNumberPerVoxel() const { return min_points_per_voxel_; }

        inline void
        setSaveLeafLayout(bool save_leaf_layout) { save_leaf_layout_ = save_leaf_layout; }

        inline bool
        getSaveLeafLayout() const { return (save_leaf_layout_); }

        inline Eigen::Vector3i
        getMinBoxCoordinates() const { return (min_b_.head<3>()); }

        inline Eigen::Vector3i
        getMaxBoxCoordinates() const { return (max_b_.head<3>()); }

        inline Eigen::Vector3i
        getNrDivisions() const { return (div_b_.head<3>()); }

        inline Eigen::Vector3i
        getDivisionMultiplier() const { return (divb_mul_.head<3>()); }

        inline int
        getCentroidIndex(const PointT &p) const
        {
            return (leaf_layout_.at((Eigen::Vector4i(static_cast<int> (floor(p.x * inverse_leaf_size_[0])),
                                                     static_cast<int> (floor(p.y * inverse_leaf_size_[1])),
                                                     static_cast<int> (floor(p.z * inverse_leaf_size_[2])), 0) - min_b_).dot(divb_mul_)));
        }

        inline std::vector<int>
        getNeighborCentroidIndices(const PointT &reference_point, const Eigen::MatrixXi &relative_coordinates) const
        {
            Eigen::Vector4i ijk(static_cast<int> (floor(reference_point.x * inverse_leaf_size_[0])),
                                static_cast<int> (floor(reference_point.y * inverse_leaf_size_[1])),
                                static_cast<int> (floor(reference_point.z * inverse_leaf_size_[2])), 0);
            Eigen::Array4i diff2min = min_b_ - ijk;
            Eigen::Array4i diff2max = max_b_ - ijk;
            std::vector<int> neighbors(relative_coordinates.cols());
            for (Eigen::Index ni = 0; ni < relative_coordinates.cols(); ni++)
            {
                Eigen::Vector4i displacement = (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();

                if ((diff2min <= displacement.array()).all() && (diff2max >= displacement.array()).all())
                    neighbors[ni] = leaf_layout_[((ijk + displacement - min_b_).dot(divb_mul_))]; 
                else
                    neighbors[ni] = -1; 
            }
            return (neighbors);
        }

        inline std::vector<int>
        getLeafLayout() const { return (leaf_layout_); }

        inline Eigen::Vector3i
        getGridCoordinates(float x, float y, float z) const
        {
            return (Eigen::Vector3i(static_cast<int> (floor(x * inverse_leaf_size_[0])),
                                    static_cast<int> (floor(y * inverse_leaf_size_[1])),
                                    static_cast<int> (floor(z * inverse_leaf_size_[2]))));
        }

        inline int
        getCentroidIndexAt(const Eigen::Vector3i &ijk) const
        {
            int idx = ((Eigen::Vector4i() << ijk, 0).finished() - min_b_).dot(divb_mul_);
            if (idx < 0 || idx >= static_cast<int> (leaf_layout_.size())) 
            {

                return (-1);
            }
            return (leaf_layout_[idx]);
        }

        inline void
        setFilterFieldName(const std::string &field_name)
        {
            filter_field_name_ = field_name;
        }

        inline std::string const
        getFilterFieldName() const
        {
            return (filter_field_name_);
        }

        inline void
        setFilterLimits(const double &limit_min, const double &limit_max)
        {
            filter_limit_min_ = limit_min;
            filter_limit_max_ = limit_max;
        }

        inline void
        getFilterLimits(double &limit_min, double &limit_max) const
        {
            limit_min = filter_limit_min_;
            limit_max = filter_limit_max_;
        }

        inline void
        setFilterLimitsNegative(const bool limit_negative)
        {
            filter_limit_negative_ = limit_negative;
        }

        inline void
        getFilterLimitsNegative(bool &limit_negative) const
        {
            limit_negative = filter_limit_negative_;
        }

        inline bool
        getFilterLimitsNegative() const
        {
            return (filter_limit_negative_);
        }

    protected:

        Eigen::Vector4f leaf_size_;

        Eigen::Array4f inverse_leaf_size_;

        bool downsample_all_data_;

        bool save_leaf_layout_;

        std::vector<int> leaf_layout_;

        Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

        std::string filter_field_name_;

        double filter_limit_min_;

        double filter_limit_max_;

        bool filter_limit_negative_;

        unsigned int min_points_per_voxel_;

        typedef typename pcl::traits::fieldList<PointT>::type FieldList;

        void
        applyFilter(PointCloud &output) override;
    };

    template<>
    class PCL_EXPORTS VoxelGridLarge<pcl::PCLPointCloud2> : public Filter<pcl::PCLPointCloud2>
    {
        using Filter<pcl::PCLPointCloud2>::filter_name_;
        using Filter<pcl::PCLPointCloud2>::getClassName;

        typedef pcl::PCLPointCloud2 PCLPointCloud2;
        typedef PCLPointCloud2::Ptr PCLPointCloud2Ptr;
        typedef PCLPointCloud2::ConstPtr PCLPointCloud2ConstPtr;

    public:

        VoxelGridLarge() :
                leaf_size_(Eigen::Vector4f::Zero()),
                inverse_leaf_size_(Eigen::Array4f::Zero()),
                downsample_all_data_(true),
                save_leaf_layout_(false),
                min_b_(Eigen::Vector4i::Zero()),
                max_b_(Eigen::Vector4i::Zero()),
                div_b_(Eigen::Vector4i::Zero()),
                divb_mul_(Eigen::Vector4i::Zero()),
                filter_field_name_(""),
                filter_limit_min_(-FLT_MAX),
                filter_limit_max_(FLT_MAX),
                filter_limit_negative_(false),
                min_points_per_voxel_(0)
        {
            filter_name_ = "VoxelGridLarge";
        }

        ~VoxelGridLarge()
        {
        }

        inline void
        setLeafSize(const Eigen::Vector4f &leaf_size)
        {
            leaf_size_ = leaf_size;

            if (leaf_size_[3] == 0)
                leaf_size_[3] = 1;

            inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
        }

        inline void
        setLeafSize(float lx, float ly, float lz)
        {
            leaf_size_[0] = lx;
            leaf_size_[1] = ly;
            leaf_size_[2] = lz;

            if (leaf_size_[3] == 0)
                leaf_size_[3] = 1;

            inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
        }

        inline Eigen::Vector3f
        getLeafSize() const { return (leaf_size_.head<3>()); }

        inline void
        setDownsampleAllData(bool downsample) { downsample_all_data_ = downsample; }

        inline bool
        getDownsampleAllData() const { return (downsample_all_data_); }

        inline void
        setMinimumPointsNumberPerVoxel(unsigned int min_points_per_voxel) { min_points_per_voxel_ = min_points_per_voxel; }

        inline unsigned int
        getMinimumPointsNumberPerVoxel() const { return min_points_per_voxel_; }

        inline void
        setSaveLeafLayout(bool save_leaf_layout) { save_leaf_layout_ = save_leaf_layout; }

        inline bool
        getSaveLeafLayout() const { return (save_leaf_layout_); }

        inline Eigen::Vector3i
        getMinBoxCoordinates() const { return (min_b_.head<3>()); }

        inline Eigen::Vector3i
        getMaxBoxCoordinates() const { return (max_b_.head<3>()); }

        inline Eigen::Vector3i
        getNrDivisions() const { return (div_b_.head<3>()); }

        inline Eigen::Vector3i
        getDivisionMultiplier() const { return (divb_mul_.head<3>()); }

        inline int
        getCentroidIndex(float x, float y, float z) const
        {
            return (leaf_layout_.at((Eigen::Vector4i(static_cast<int> (floor(x * inverse_leaf_size_[0])),
                                                     static_cast<int> (floor(y * inverse_leaf_size_[1])),
                                                     static_cast<int> (floor(z * inverse_leaf_size_[2])),
                                                     0)
                                     - min_b_).dot(divb_mul_)));
        }

        inline std::vector<int>
        getNeighborCentroidIndices(float x, float y, float z, const Eigen::MatrixXi &relative_coordinates) const
        {
            Eigen::Vector4i ijk(static_cast<int> (floor(x * inverse_leaf_size_[0])),
                                static_cast<int> (floor(y * inverse_leaf_size_[1])),
                                static_cast<int> (floor(z * inverse_leaf_size_[2])), 0);
            Eigen::Array4i diff2min = min_b_ - ijk;
            Eigen::Array4i diff2max = max_b_ - ijk;
            std::vector<int> neighbors(relative_coordinates.cols());
            for (Eigen::Index ni = 0; ni < relative_coordinates.cols(); ni++)
            {
                Eigen::Vector4i displacement = (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();

                if ((diff2min <= displacement.array()).all() && (diff2max >= displacement.array()).all())
                    neighbors[ni] = leaf_layout_[((ijk + displacement - min_b_).dot(divb_mul_))]; 
                else
                    neighbors[ni] = -1; 
            }
            return (neighbors);
        }

        inline std::vector<int>
        getNeighborCentroidIndices(float x, float y, float z, const std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > &relative_coordinates) const
        {
            Eigen::Vector4i ijk(static_cast<int> (floorf(x * inverse_leaf_size_[0])), static_cast<int> (floorf(y * inverse_leaf_size_[1])),
                                static_cast<int> (floorf(z * inverse_leaf_size_[2])), 0);
            std::vector<int> neighbors;
            neighbors.reserve(relative_coordinates.size());
            for (const auto &relative_coordinate : relative_coordinates)
                neighbors.push_back(leaf_layout_[(ijk + (Eigen::Vector4i() << relative_coordinate, 0).finished() - min_b_).dot(divb_mul_)]);
            return (neighbors);
        }

        inline std::vector<int>
        getLeafLayout() const { return (leaf_layout_); }

        inline Eigen::Vector3i
        getGridCoordinates(float x, float y, float z) const
        {
            return (Eigen::Vector3i(static_cast<int> (floor(x * inverse_leaf_size_[0])),
                                    static_cast<int> (floor(y * inverse_leaf_size_[1])),
                                    static_cast<int> (floor(z * inverse_leaf_size_[2]))));
        }

        inline int
        getCentroidIndexAt(const Eigen::Vector3i &ijk) const
        {
            int idx = ((Eigen::Vector4i() << ijk, 0).finished() - min_b_).dot(divb_mul_);
            if (idx < 0 || idx >= static_cast<int> (leaf_layout_.size())) 
            {

                return (-1);
            }
            return (leaf_layout_[idx]);
        }

        inline void
        setFilterFieldName(const std::string &field_name)
        {
            filter_field_name_ = field_name;
        }

        inline std::string const
        getFilterFieldName() const
        {
            return (filter_field_name_);
        }

        inline void
        setFilterLimits(const double &limit_min, const double &limit_max)
        {
            filter_limit_min_ = limit_min;
            filter_limit_max_ = limit_max;
        }

        inline void
        getFilterLimits(double &limit_min, double &limit_max) const
        {
            limit_min = filter_limit_min_;
            limit_max = filter_limit_max_;
        }

        inline void
        setFilterLimitsNegative(const bool limit_negative)
        {
            filter_limit_negative_ = limit_negative;
        }

        inline void
        getFilterLimitsNegative(bool &limit_negative) const
        {
            limit_negative = filter_limit_negative_;
        }

        inline bool
        getFilterLimitsNegative() const
        {
            return (filter_limit_negative_);
        }

    protected:

        Eigen::Vector4f leaf_size_;

        Eigen::Array4f inverse_leaf_size_;

        bool downsample_all_data_;

        bool save_leaf_layout_;

        std::vector<int> leaf_layout_;

        Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

        std::string filter_field_name_;

        double filter_limit_min_;

        double filter_limit_max_;

        bool filter_limit_negative_;

        unsigned int min_points_per_voxel_;

        void
        applyFilter(PCLPointCloud2 &output) override;
    };
}

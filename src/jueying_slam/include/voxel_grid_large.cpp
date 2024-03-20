

#include <iostream>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/voxel_grid_large.h>

#include <pcl/filters/passthrough.h>

typedef Eigen::Array<size_t, 4, 1> Array4size_t;

struct cloud_point_index_idx
{
    unsigned int idx;
    unsigned int cloud_point_index;

    cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}

    bool operator<(const cloud_point_index_idx &p) const { return (idx < p.idx); }
};

template<typename PointT>
void
pcl::VoxelGridLarge<PointT>::applyFilter(PointCloud &output)
{

    if (!input_)
    {
        PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName().c_str());
        output.width = output.height = 0;
        output.points.clear();
        return;
    }

    output.height = 1;                    
    output.is_dense = true;                 

    Eigen::Vector4f min_p, max_p;

    if (!filter_field_name_.empty()) 
        getMinMax3D<PointT>(input_, *indices_, filter_field_name_, static_cast<float> (filter_limit_min_), static_cast<float> (filter_limit_max_), min_p, max_p,
                            filter_limit_negative_);
    else
        getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
    int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

    if ((dx * dy * dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
    {

        pcl::PointCloud<PointT> cloud1, cloud2,cloud1f, cloud2f;
        pcl::PassThrough<PointT> pass;

        if (dx > dy && dx > dz)
        {
            pass.setInputCloud(input_);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(min_p[0], min_p[0] + (max_p[0] - min_p[0]) / 2);
            pass.filter(cloud1);

            pass.setFilterLimitsNegative(true);
            pass.filter(cloud2);
        } else if (dy > dx && dy > dz)
        {
            pass.setInputCloud(input_);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(min_p[1], min_p[1] + (max_p[1] - min_p[1]) / 2);
            pass.filter(cloud1);

            pass.setFilterLimitsNegative(true);
            pass.filter(cloud2);
        } else
        {
            pass.setInputCloud(input_);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_p[2], min_p[2] + (max_p[2] - min_p[2]) / 2);
            pass.filter(cloud1);

            pass.setFilterLimitsNegative(true);
            pass.filter(cloud2);
        }

        VoxelGridLarge voxelGridLarge = *this;
        voxelGridLarge.setInputCloud(cloud1.makeShared());
        voxelGridLarge.filter(cloud1f);

        voxelGridLarge.setInputCloud(cloud2.makeShared());
        voxelGridLarge.filter(cloud2f);

        output = cloud1f + cloud2f;
        return;
    }

    min_b_[0] = static_cast<int> (floor(min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (floor(max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (floor(min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int> (floor(max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int> (floor(min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int> (floor(max_p[2] * inverse_leaf_size_[2]));

    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;

    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve(indices_->size());

    if (!filter_field_name_.empty())
    {

        std::vector<pcl::PCLPointField> fields;
        int distance_idx = pcl::getFieldIndex(*input_, filter_field_name_, fields);
        if (distance_idx == -1)
            PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName().c_str(), distance_idx);

        for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
        {
            if (!input_->is_dense)

                if (!std::isfinite(input_->points[*it].x) ||
                    !std::isfinite(input_->points[*it].y) ||
                    !std::isfinite(input_->points[*it].z))
                    continue;

            const uint8_t *pt_data = reinterpret_cast<const uint8_t *> (&input_->points[*it]);
            float distance_value = 0;
            memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof(float));

            if (filter_limit_negative_)
            {

                if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
                    continue;
            } else
            {

                if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
                    continue;
            }

            int ijk0 = static_cast<int> (floor(input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
            int ijk1 = static_cast<int> (floor(input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
            int ijk2 = static_cast<int> (floor(input_->points[*it].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
            index_vector.emplace_back(static_cast<unsigned int> (idx), *it);
        }
    }

    else
    {

        for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
        {
            if (!input_->is_dense)

                if (!std::isfinite(input_->points[*it].x) ||
                    !std::isfinite(input_->points[*it].y) ||
                    !std::isfinite(input_->points[*it].z))
                    continue;

            int ijk0 = static_cast<int> (floor(input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
            int ijk1 = static_cast<int> (floor(input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
            int ijk2 = static_cast<int> (floor(input_->points[*it].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
            index_vector.emplace_back(static_cast<unsigned int> (idx), *it);
        }
    }

    std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

    unsigned int total = 0;
    unsigned int index = 0;

    std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;

    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size())
    {
        unsigned int i = index + 1;
        while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
            ++i;
        if (i - index >= min_points_per_voxel_)
        {
            ++total;
            first_and_last_indices_vector.emplace_back(index, i);
        }
        index = i;
    }

    output.points.resize(total);
    if (save_leaf_layout_)
    {
        try
        {

            uint32_t new_layout_size = div_b_[0] * div_b_[1] * div_b_[2];

            uint32_t reinit_size = std::min(static_cast<unsigned int> (new_layout_size), static_cast<unsigned int> (leaf_layout_.size()));
            for (uint32_t i = 0; i < reinit_size; i++)
            {
                leaf_layout_[i] = -1;
            }
            leaf_layout_.resize(new_layout_size, -1);
        }
        catch (std::bad_alloc &)
        {
            throw PCLException("VoxelGridLarge bin size is too low; impossible to allocate memory for layout",
                               "voxel_grid.hpp", "applyFilter");
        }
        catch (std::length_error &)
        {
            throw PCLException("VoxelGridLarge bin size is too low; impossible to allocate memory for layout",
                               "voxel_grid.hpp", "applyFilter");
        }
    }

    index = 0;
    for (const auto &cp : first_and_last_indices_vector)
    {

        unsigned int first_index = cp.first;
        unsigned int last_index = cp.second;

        if (save_leaf_layout_)
            leaf_layout_[index_vector[first_index].idx] = index;

        if (!downsample_all_data_)
        {
            Eigen::Vector4f centroid(Eigen::Vector4f::Zero ());

            for (unsigned int li = first_index; li < last_index; ++li)
                centroid += input_->points[index_vector[li].cloud_point_index].getVector4fMap();

            centroid /= static_cast<float> (last_index - first_index);
            output.points[index].getVector4fMap() = centroid;
        } else
        {
            CentroidPoint<PointT> centroid;

            for (unsigned int li = first_index; li < last_index; ++li)
                centroid.add(input_->points[index_vector[li].cloud_point_index]);

            centroid.get(output.points[index]);
        }

        ++index;
    }
    output.width = static_cast<uint32_t> (output.points.size());
}

#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#define PCL_INSTANTIATE_VoxelGridLarge(T) template class PCL_EXPORTS pcl::VoxelGridLarge<T>;

PCL_INSTANTIATE(VoxelGridLarge, PCL_XYZ_POINT_TYPES)
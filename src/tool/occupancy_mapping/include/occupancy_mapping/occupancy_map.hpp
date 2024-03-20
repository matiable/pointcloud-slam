#ifndef OCCUPANCY_MAP_H_
#define OCCUPANCY_MAP_H_

#include "utility.hpp"

namespace occupancy_mapping
{

    class OccupancyMap2D
    {
    public:
        struct MapParams
        {
            double log_occ, log_free;
            double resolution;
        };

        OccupancyMap2D(const MapParams &params) : map_params_(params), occupancy_map_root_(nullptr) {}

        struct MapTreeNode
        {
            int level;      //节点的层级，分辨率最小层为0
            double logit;   //表示概率，初始为0，占据为正，空闲为负，更新时需加上或减去一个值
            int position_x; //当前节点栅格左下角位置，则当前节点栅格范围为(position_x, position_x+resolution*2^level)(x向右，y向上)
            int position_y;
            MapTreeNode *child[4];
            MapTreeNode() : level(0), logit(0.0), position_x(0), position_y(0)
            {
                child[0] = nullptr;
                child[1] = nullptr;
                child[2] = nullptr;
                child[3] = nullptr;
            }
            MapTreeNode(int level_, double logit_, int x_, int y_) : level(level_), logit(logit_), position_x(x_), position_y(y_)
            {
                child[0] = nullptr;
                child[1] = nullptr;
                child[2] = nullptr;
                child[3] = nullptr;
            }
            MapTreeNode(int level_, int x_, int y_) : level(level_), logit(0.0), position_x(x_), position_y(y_)
            {
                child[0] = nullptr;
                child[1] = nullptr;
                child[2] = nullptr;
                child[3] = nullptr;
            }
        };

        GridIndex ConvertWorld2GridIndex(double x, double y)
        {
            GridIndex index;
            index.x = int(x / map_params_.resolution);
            index.y = int(y / map_params_.resolution);
            return index;
        }

        void deleteNode(MapTreeNode *node){
            if(node != nullptr){
                for (int i = 0; i < 4; i++){
                    deleteNode(node->child[i]);
                }
                delete node;
                return;
            }
            return;
        }

        void initializeMap(std::vector<double> robot_pose)
        {
            GridIndex robot_index = ConvertWorld2GridIndex(robot_pose[3], robot_pose[4]);
            // 先delete原有地图
            deleteNode(occupancy_map_root_);
            occupancy_map_root_ = new MapTreeNode(0, robot_index.x, robot_index.y);
        }

        //函数，查找节点node在根节点的哪个子节点中，可递归查找
        int findChild(MapTreeNode *node, GridIndex index)
        {
            int child_length = pow(2, (node->level - 1));
            if (node->level == 0)
            {
                if (node->position_x == index.x && node->position_y == index.y)
                {
                    return 0;
                }
                else
                {
                    return 4;
                }
            }
            if (index.x >= node->position_x && index.x < node->position_x + child_length && index.y >= node->position_y && index.y < node->position_y + child_length)
            {
                return 2;
            }
            else if (index.x >= node->position_x && index.x < node->position_x + child_length && index.y >= node->position_y + child_length && index.y < node->position_y + 2 * child_length)
            {
                return 1;
            }
            else if (index.x >= node->position_x + child_length && index.x < node->position_x + 2 * child_length && index.y >= node->position_y && index.y < node->position_y + child_length)
            {
                return 3;
            }
            else if (index.x >= node->position_x + child_length && index.x < node->position_x + 2 * child_length && index.y >= node->position_y + child_length && index.y < node->position_y + 2 * child_length)
            {
                return 0;
            }
            else
            {
                return 4;
            }
        }

        void extendMap(GridIndex index)
        {
            MapTreeNode *root_temp;
            root_temp = new MapTreeNode();
            root_temp->level = occupancy_map_root_->level + 1;
            int grid_length = pow(2, occupancy_map_root_->level);
            if (index.x >= occupancy_map_root_->position_x && index.y >= occupancy_map_root_->position_y + grid_length)
            {
                root_temp->child[2] = occupancy_map_root_;
                root_temp->position_x = occupancy_map_root_->position_x;
                root_temp->position_y = occupancy_map_root_->position_y;
            }
            else if (index.x < occupancy_map_root_->position_x && index.y >= occupancy_map_root_->position_y)
            {
                root_temp->child[3] = occupancy_map_root_;
                root_temp->position_x = occupancy_map_root_->position_x - grid_length;
                root_temp->position_y = occupancy_map_root_->position_y;
            }
            else if (index.x < occupancy_map_root_->position_x + grid_length && index.y < occupancy_map_root_->position_y)
            {
                root_temp->child[0] = occupancy_map_root_;
                root_temp->position_x = occupancy_map_root_->position_x - grid_length;
                root_temp->position_y = occupancy_map_root_->position_y - grid_length;
            }
            else if (index.x >= occupancy_map_root_->position_x + grid_length && index.y < occupancy_map_root_->position_y + grid_length)
            {
                root_temp->child[1] = occupancy_map_root_;
                root_temp->position_x = occupancy_map_root_->position_x;
                root_temp->position_y = occupancy_map_root_->position_y - grid_length;
            }
            else
            {
                std::cout<<"Wrong in extendMap"<<std::endl;
            }
            occupancy_map_root_ = root_temp;
        }

        void updateGrid(MapTreeNode *node, GridIndex index, grid_status status)
        {
            int child_index = findChild(node, index); //寻找目标index在哪个子区域
            int child_length = pow(2, (node->level - 1));
            if (child_index >= 0 && child_index <= 3) //如果目标index在本节点四个子节点区域内
            {
                if (node->level != 0)
                { //如果层级不等于0
                    MapTreeNode *childptr = node->child[child_index];
                    if (childptr == nullptr)
                    {
                        childptr = new MapTreeNode(node->level - 1, node->position_x + child_length * (1 ^ (((child_index >> 1) & 1) ^ (child_index & 1))), node->position_y + child_length * (((child_index >> 1) & 1) ^ 1));
                        node->child[child_index] = childptr;
                    }
                    updateGrid(childptr, index, status);
                }
                else
                {
                    if (status == occ_status)
                    {
                        node->logit += map_params_.log_occ;
                    }
                    else if (status == free_status)
                    {
                        node->logit += map_params_.log_free;
                    }
                    else
                    {
                        std::cout<<"unknown"<<std::endl;
                    }
                }
            }
            else //如果目标index不在本节点区域内
            {
                extendMap(index); //扩展一层节点
                updateGrid(occupancy_map_root_, index, status);
            }
        }

        MapTreeNode *getMapRoot()
        {
            return occupancy_map_root_;
        }

        double getMapResolution()
        {
            return map_params_.resolution;
        }

    protected:
        MapTreeNode *occupancy_map_root_;
        MapParams map_params_;
    };

}

#endif
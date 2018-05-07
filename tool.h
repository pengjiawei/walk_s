//
// Created by root on 18-5-2.
//

#ifndef LOOKUP_TOOL_H
#define LOOKUP_TOOL_H

#include <vector>
//occ value not costmap value
static const char UNKNOWN = -1;
static const char OBSTACLE = 100;
static const char INSCRIBED_INFLATED_OBSTACLE = 99;
static const char FREE_SPACE = 0;

//static const unsigned char NO_INFORMATION = 255;
//static const unsigned char LETHAL_OBSTACLE = 254;
//static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
//static const unsigned char FREE_SPACE = 0;
int size_x,size_y;
inline int left(int point){
    if((point % size_x != 0)){
        return point-1;
    }
    return -1;
}
inline int upleft(int point){
    if((point % size_x != 0) && (point >= (int)size_x)){
        return point-1-size_x;
    }
    return -1;

}
inline int up(int point){
    if(point >= (int)size_x){
        return point-size_x;
    }
    return -1;
}
inline int upright(int point){
    if((point >= (int)size_x) && ((point + 1) % (int)size_x != 0)){
        return point-size_x+1;
    }
    return -1;
}
inline int right(int point){
    if((point + 1) % size_x != 0){
        return point+1;
    }
    return -1;

}
inline int downright(int point){
    if(((point + 1) % size_x != 0) && ((point/size_x) < (size_y-1))){
        return point+size_x+1;
    }
    return -1;

}
inline int down(int point){
    if((point/size_x) < (size_y-1)){
        return point+size_x;
    }
    return -1;

}
inline int downleft(int point){
    if(((point/size_x) < (size_y-1)) && (point % size_x != 0)){
        return point+size_x-1;
    }
    return -1;

}


void get_adjacent_point(int point, int *adjacent_point) {
    adjacent_point[0] = left(point);  //left
    adjacent_point[1] = up(point); //up
    adjacent_point[2] = right(point);  //right
    adjacent_point[3] = down(point); //down
    adjacent_point[4] = upleft(point); //upleft
    adjacent_point[5] = upright(point);  //upright
    adjacent_point[6] = downright(point);  //downright
    adjacent_point[7] = downleft(point);  //downleft
}
bool is_valid(int point) {
    return  point > 0 && point < size_x * size_y;
}


bool isFrontier(nav_msgs::OccupancyGrid& occ,int index){
    if(occ.data[index] == FREE_SPACE){
        int adjacent[8];
        get_adjacent_point(index,adjacent);
        for (int i = 0; i < 8; ++i) {
            if (is_valid(adjacent[i])){
                if (occ.data[adjacent[i]] == UNKNOWN){
                    int no_info_count = 0;
                    int no_inf_adj_points[8];
                    get_adjacent_point(adjacent[i],no_inf_adj_points);
                    for (int j = 0; j < 8; ++j) {
                        if (is_valid(no_inf_adj_points[j]) && occ.data[no_inf_adj_points[j]] == UNKNOWN){
                            ++no_info_count;
                            if (no_info_count > 2){
                                return true;
                            }

                        }
                    }
                }
            }
        }
    }
    return false;
}

void findFrontier(nav_msgs::OccupancyGrid& occ,std::vector<int>& frontiers){
    for (int i = 0; i < size_x * size_y; ++i) {
        if(isFrontier(occ,i)){
            frontiers.push_back(i);
        }
    }
    printf("frontier size = %d\n",frontiers.size());
}

#endif //LOOKUP_TOOL_H

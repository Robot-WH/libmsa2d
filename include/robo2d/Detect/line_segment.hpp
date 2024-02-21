void ObstacleExtractor::detectSegments(const PointSet &point_set)
{
    std::cout << "detect segment now ,num points is: " << point_set.num_points << endl;
    if (point_set.num_points < p_min_group_points_)
        return;

    Segment segment(*point_set.begin, *point_set.end); // Use Iterative End Point Fit

    // if (p_use_split_and_merge_)
    //     segment = fitSegment(point_set);

    PointIterator set_divider;
    double max_distance = 0.0;
    double distance     = 0.0;

    int split_index = 0; // Natural index of splitting point (counting from 1)
    int point_index = 0; // Natural index of current point in the set

    // Seek the point of division
    // 离首尾直线段最远的作为分段点
    for (PointIterator point = point_set.begin; point != point_set.end; ++point) {
        ++point_index;

        if ((distance = segment.distanceTo(*point)) >= max_distance) {
            // double r = (*point).length();

            // todo:这里要对照论文调整，这里的值给的小一些提取的线段会更多；
            // if (distance > p_max_split_distance_ + r * p_distance_proportion_) {
            if (distance > 0.05) {
                max_distance = distance;
                set_divider  = point;
                split_index  = point_index;
            }
        }
    }

    // Split the set only if the sub-groups are not 'small'
    // 被分割的点集中点的个数不能少于3个 todo:可以调整
    if (max_distance > 0.0 
            && split_index > p_min_group_points_ 
            && split_index < point_set.num_points - p_min_group_points_) {
        set_divider = input_points_.insert(set_divider, *set_divider); // Clone the dividing point for each group

        PointSet subset1, subset2;
        subset1.begin      = point_set.begin;
        subset1.end        = set_divider;
        subset1.num_points = split_index;
        subset1.is_visible = point_set.is_visible;

        subset2.begin      = ++set_divider;
        subset2.end        = point_set.end;
        subset2.num_points = point_set.num_points - split_index;
        subset2.is_visible = point_set.is_visible;

        detectSegments(subset1);
        detectSegments(subset2);
    } else { // Add the segment
        if (!p_use_split_and_merge_) {
            segment = fitSegment(point_set); // 使用最小二乘拟合直线
        }
        segments_.push_back(segment);
    }
}

/**
 * @brief 合并直线段，根据直线段间的法向距离或者真实距离
 * 
 */
void ObstacleExtractor::mergeSegments()
{
    for (auto i = segments_.begin(); i != segments_.end(); ++i) {
        for (auto j = i; j != segments_.end(); ++j) {
            Segment merged_segment;

            if (compareSegments(*i, *j, merged_segment)) {
                auto temp_itr = segments_.insert(i, merged_segment);
                segments_.erase(i);
                segments_.erase(j);
                i = --temp_itr; // Check the new segment against others
                break;
            }
        }
    }
}

bool ObstacleExtractor::compareSegments(const Segment &s1, const Segment &s2, Segment &merged_segment)
{
    if (&s1 == &s2)
        return false;

    // Segments must be provided counter-clockwise
    if (s1.first_point.cross(s2.first_point) < 0.0)
        return compareSegments(s2, s1, merged_segment);

    if (checkSegmentsProximity(s1, s2)) {
        vector<PointSet> point_sets;
        point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
        point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

        Segment segment = fitSegment2(point_sets);

        if (checkSegmentsCollinearity(segment, s1, s2)) {
            merged_segment = segment;
            return true;
        }
    }

    return false;
}

bool ObstacleExtractor::checkSegmentsProximity(const Segment &s1, const Segment &s2)
{
    //   return (s1.trueDistanceTo(s2.first_point) < p_max_merge_separation_ ||
    //           s1.trueDistanceTo(s2.last_point)  < p_max_merge_separation_ ||
    //           s2.trueDistanceTo(s1.first_point) < p_max_merge_separation_ ||
    //           s2.trueDistanceTo(s1.last_point)  < p_max_merge_separation_);
    // todo: 这里的相邻点距离阈值要调整
    return (s1.trueDistanceTo(s2.first_point) < 0.05 ||
            s1.trueDistanceTo(s2.last_point) < 0.05 ||
            s2.trueDistanceTo(s1.first_point) < 0.05 ||
            s2.trueDistanceTo(s1.last_point) < 0.05);
}

bool ObstacleExtractor::checkSegmentsCollinearity(const Segment &segment, const Segment &s1, const Segment &s2)
{
    return (segment.distanceTo(s1.first_point) < 0.05 &&
            segment.distanceTo(s1.last_point) < 0.05 &&
            segment.distanceTo(s2.first_point) < 0.05 &&
            segment.distanceTo(s2.last_point) < 0.05);
}
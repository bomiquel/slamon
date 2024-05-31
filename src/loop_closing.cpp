#include "loop_closing.hpp"


LoopClosing::LoopClosing(int min_lc_inliers,
                         bool enable_debugging,
                         const std::string& w_path)
{
    w_path_ = w_path;
    min_lc_inliers_ = min_lc_inliers;
    enable_debugging_ = enable_debugging;
}
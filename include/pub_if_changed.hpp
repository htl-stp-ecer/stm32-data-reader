//
// Created by tobias on 7/16/25.
//

#pragma once
#include <lcm/lcm-cpp.hpp>
#include <cstring>

template <class Msg>
class PubIfChanged
{
    lcm::LCM* lc_;
    std::string chan_;
    Msg last_{};
    bool first_ = true;

public:
    PubIfChanged() : lc_(nullptr) {}
    
    PubIfChanged(lcm::LCM& lc, std::string chan)
        : lc_(&lc), chan_(std::move(chan))
    {
    }

    void operator()(const Msg& cur)
    {
        if (!lc_) return;
        if (first_ || std::memcmp(&cur, &last_, sizeof(Msg)) != 0)
        {
            lc_->publish(chan_, &cur);
            last_ = cur;
            first_ = false;
        }
    }
};

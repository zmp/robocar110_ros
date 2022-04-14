/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "../include/param_tools/param_tools.hpp"

namespace param_tools
{
Subscriber::~Subscriber()
{
    if (parameter && parameter.use_count() == 1) {
        instance().unsubscribe(this);
    }
}
}  // namespace param_tools

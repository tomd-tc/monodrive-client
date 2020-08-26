// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#include <string>
#include <vector>

#if defined UE_BUILD_DEBUG || defined UE_BUILD_DEVELOPMENT || defined UE_BUILD_TEST || defined UE_BUILD_SHIPPING
#define UE_BUILD
#endif

#ifndef UE_BUILD
#define MONODRIVECORE_API
#endif

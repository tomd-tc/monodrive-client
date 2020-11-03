// Copyright (C) 2017-2020, monoDrive, LLC. All Rights Reserved.
#pragma once

#if defined UE_BUILD_DEBUG || defined UE_BUILD_DEVELOPMENT || defined UE_BUILD_TEST || defined UE_BUILD_SHIPPING
#else
#define MONODRIVECORE_API
#endif



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

#ifdef UE_BUILD
#define SENSOR_CONFIG USTRUCT(BlueprintType)
#define SENSOR_PROPERTY(category) UPROPERTY(Category = category, EditAnywhere, BlueprintReadWrite)
#else
#define SENSOR_CONFIG
#define SENSOR_PROPERTY(category)
#endif

#ifdef UE_BUILD
#define STRING_CLS FString
#define NAME_CLS FName
#define VECTOR_CLS TArray
#else
#define STRING_CLS std::string
#define NAME_CLS std::string
#define VECTOR_CLS std::vector
#endif

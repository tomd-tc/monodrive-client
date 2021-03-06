// monoDrive Client utils
#pragma once

template<typename T, typename... Args>
std::unique_ptr<T> static make_unique(Args&&... args)
{
  // helper for old c++11 standard
  // h/t https://stackoverflow.com/a/24609331
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

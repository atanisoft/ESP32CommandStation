/**
 * \file MakeUnique.hxx
 *
 * C++11 version of std::make_unique which is only available from c++14 or
 * later.
 * 
 * This is based on https://isocpp.org/files/papers/N3656.txt.
 * 
 * The __cplusplus constant reference is from:
 * http://www.open-std.org/JTC1/SC22/WG21/docs/papers/2014/n3938.html
 * 
 */

// Check if we are building with less than C++14 and if so we need to define
// the std::make_unique() API.
#if __cplusplus < 201402L

#include <memory>
#include <type_traits>
#include <utility>

namespace std
{

template <typename T, typename... Args>
unique_ptr<T> make_unique_helper(false_type, Args&&... args)
{
  return unique_ptr<T>(new T(forward<Args>(args)...));
}

template <typename T, typename... Args>
unique_ptr<T> make_unique_helper(true_type, Args&&... args)
{
   static_assert(extent<T>::value == 0,
       "make_unique<T[N]>() is forbidden, please use make_unique<T[]>().");

   typedef typename remove_extent<T>::type U;
   return unique_ptr<T>(new U[sizeof...(Args)]{forward<Args>(args)...});
}

template <typename T, typename... Args>
unique_ptr<T> make_unique(Args&&... args)
{
   return make_unique_helper<T>(is_array<T>(), forward<Args>(args)...);
}

}

#endif // __cplusplus < 201402L
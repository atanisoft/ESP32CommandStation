/** \copyright
 * Copyright (c) 2015, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file OptionalArgs.hxx
 *
 * Constexpr structure for storing a list of optional arguments to a function
 * that needs to be optimized at compile time.
 *
 * @author Balazs Racz
 * @date 24 November 2015
 */

#ifndef _UTILS_OPTIONALARGS_HXX_
#define _UTILS_OPTIONALARGS_HXX_

/// Used as an argument to the get(Fetcher) function in the OptionalArg
/// implementation to select which entry to retrieve.
template <typename DATA_TYPE, int N> class Fetcher
{
public:
    constexpr Fetcher()
    {
    }
};

/// Used as an argument to the constructor of the OptionalArg implementation to
/// represent that a specific argument has to be overridden with a given value.
template <typename DATA_TYPE, int N, DATA_TYPE defval> class Specifier
{
public:
    /// Constructor. Specifies a particular value. @param d is the value the
    /// client tells us.
    constexpr Specifier(const DATA_TYPE d)
        : d_(d)
    {
    }

    /// Type link to the respective fetcher.
    typedef Fetcher<DATA_TYPE, N> FetcherType;
    /// Type of data held internally.
    typedef DATA_TYPE data_type;
    /// Default value when the customer does not specify it.
    static constexpr DATA_TYPE default_value()
    {
        return defval;
    }
    /// Shuttled argument value.
    DATA_TYPE d_;
};

/// Use this macro in the Defs structure of an optional args instance to add an
/// optional argument.
///
/// @param SpecName Name by which the user will specify the given argument.
/// Usually capitalized like a class.
/// @param function_name function name by which to access the argument in the
/// final structure. Usually lower_case like a function name.
/// @param DataType C++ data type of the argument
/// @param N A unique integer assigned to this argument in the current
/// optionalargs instance (and its base classes).
/// @param DEF Default value that should be returned from the structure if the
/// user does not specify it.
#define DECLARE_OPTIONALARG(SpecName, function_name, DataType, N, DEF)         \
    using SpecName = Specifier<DataType, N, (DEF)>;                            \
    using SpecName##Get = Fetcher<DataType, N>;                                \
    static constexpr int check_arguments_are_valid(const SpecName s)           \
    {                                                                          \
        return 0;                                                              \
    }

/// Use this macro in the final optionalargs structure. Each entry in the Defs
/// structure should have a definition with matching options.
///
/// @param SpecName same as in @ref DECLARE_OPTIONALARG
/// @param function_name same as in @ref DECLARE_OPTIONALARG
/// @param DataType same as in @ref DECLARE_OPTIONALARG
#define DEFINE_OPTIONALARG(SpecName, function_name, DataType)                  \
    constexpr DataType function_name() const                                   \
    {                                                                          \
        return get(SpecName##Get());                                           \
    }                                                                          \
    constexpr bool has_##function_name() const                                 \
    {                                                                          \
        return has(SpecName##Get());                                           \
    }

/// Declares that a recursive class template is coming.
template <class Decl, typename... Args> class OptionalArg;

/// Terminates the class recursion template.
///
/// The constructor is used to check that the customer has specified only known
/// parameters.
///
/// The get function throws an error deterministically, because if the compiler
/// gets this deep in the get function, then none of the actual data carrying
/// components matched on the type. This means that the customer is trying to
/// fetch something we didn't store.
template <class Decl> class OptionalArg<Decl> : public Decl
{
public:
    /// Constructor. @param args arguments.
    template <typename... Args>
    constexpr OptionalArg(Args... args)
        : check_(check_all_args(args...))
    {
    }

    /// End of template recursion. Fails to link. @param f unknown argument
    /// specification. @return nothing
    template <class FF> constexpr FF get(const FF f) const
    {
        return tried_to_get_unknown_argument() ? f : FF();
    }

    /// End of template recursion. Fails to link. @param f unknown argument
    /// specification. @return nothing.
    template <class FF> constexpr bool has(const FF f) const
    {
        return tried_to_get_unknown_argument();
    }

private:
    using Decl::check_arguments_are_valid;
    /// End of recursion with no leftover arguments.
    static constexpr int check_arguments_are_valid(const OptionalArg &a)
    {
        return 0;
    }

    /// Beginning of recursion..
    template <typename A, typename... Args>
    static constexpr int check_all_args(const A a, Args... args)
    {
        return check_arguments_are_valid(a) + check_all_args(args...);
    }

    /// End of recursion.
    static constexpr int check_all_args()
    {
        return 0;
    }

    /// unimplemented, causes a link error. Do not ever implement it.
    static bool tried_to_get_unknown_argument();

    /// This is here only to force computing the check_all_args at compile time
    /// (constexpr!).
    const int check_;
};

/// Template recursion entry. We have as many instances of this class in the
/// inheritance stack as the number of data elements we need to carry. Each of
/// these classes stores one single element in the private variable d_.
///
/// The constructor picks out the specifier for the current entry to fill in
/// the current storage. All arguments are forwarded to the base class. This
/// allows arbitrary order of the specified arguments; it is the responsibility
/// of the innermost class to check for spurious arguments.
///
/// The get method either recognizes the Fetcher argument as referring to the
/// current entry, or forwards it to the parent class.
template <typename Decl, typename Specifier, typename... TArgs>
class OptionalArg<Decl, Specifier, TArgs...>
    : public OptionalArg<Decl, TArgs...>
{
public:
    /// This is the type used by the customer to set the value in the
    /// initializer list.
    typedef Specifier specifier_type;
    /// This is the type we use internally to tag and fetch the value.
    typedef typename Specifier::FetcherType fetcher_type;
    /// The type of the actually stored data. Should be POD.
    typedef typename Specifier::data_type data_type;
    /// Recursion, with all other arguments.
    using Base = OptionalArg<Decl, TArgs...>;

    /// Constructor. @param args specifies the values to store.
    template <typename... Args>
    constexpr OptionalArg(Args... args)
        : Base(args...)
        , d_(GetFromArgs(args...))
        , has_(HasFromArgs(args...))
    {
    }

    /// Constructor ending the recursion.
    constexpr OptionalArg()
        : d_(GetFromArgs())
        , has_(false)
    {
    }

    /// @return the data stored.
    constexpr data_type get(const fetcher_type) const
    {
        return d_;
    }

    /// @return whether we have stored data or only the default.
    constexpr bool has(const fetcher_type) const
    {
        return has_;
    }

    /// Needed due to templated base class; the public inheritance is not
    /// enough.
    using Base::get;
    using Base::has;

private:
    /// This template gets instantiated when the first argument is for us.
    template <typename... Args>
    static constexpr data_type GetFromArgs(
        const specifier_type spec, Args... args)
    {
        return spec.d_;
    }

    /// Decides whether we have the current argument (from the original list or
    /// arguments). @return true if we have specified the argument.
    template <typename... Args>
    static constexpr bool HasFromArgs(const specifier_type spec, Args... args)
    {
        return true;
    }

    /// This template gets instantiated for a copy constructor: when the
    /// argument is already an OptionalArg (or reference to it). @param me is
    /// the copy constructor argument. @param args are the further arguments.
    template <typename U, typename... Args>
    static constexpr typename std::enable_if<
        std::is_convertible<typename std::add_lvalue_reference<U>::type,
            typename std::add_lvalue_reference<Base>::type>::value,
        data_type>::type
    GetFromArgs(const U me, Args... args)
    {
        return me.get(fetcher_type());
    }
    /// This template gets instantiated for a copy constructor: when the
    /// argument is already an OptionalArg (or reference to it).
    template <typename U, typename... Args>
    static constexpr typename std::enable_if<
        std::is_convertible<typename std::add_lvalue_reference<U>::type,
            typename std::add_lvalue_reference<Base>::type>::value,
        bool>::type
    HasFromArgs(const U me, Args... args)
    {
        return me.has(fetcher_type());
    }

    /// This template gets instantiated only if the argument is not an
    /// OptionalArg (hence not called from the copy constructor) and not a
    /// Specifier for the current entry. Then we just ignore the first arg and
    /// recurse into the rest of them.  @param t is the argument to
    /// ignore. @param args are the further arguments.
    template <typename T, typename... Args>
    static constexpr typename std::enable_if<
        !std::is_convertible<typename std::add_lvalue_reference<T>::type,
            typename std::add_lvalue_reference<Base>::type>::value,
        data_type>::type
    GetFromArgs(const T t, Args... args)
    {
        return GetFromArgs(args...);
    }
    /// This template gets instantiated only if the argument is not an
    /// OptionalArg (hence not called from the copy constructor) and not a
    /// Specifier for the current entry. Then we just ignore the first arg and
    /// recurse into the rest of them.
    template <typename T, typename... Args>
    static constexpr typename std::enable_if<
        !std::is_convertible<typename std::add_lvalue_reference<T>::type,
            typename std::add_lvalue_reference<Base>::type>::value,
        bool>::type
    HasFromArgs(const T t, Args... args)
    {
        return HasFromArgs(args...);
    }

    /// If we've run out of all arguments, we take the default value.
    static constexpr data_type GetFromArgs()
    {
        return specifier_type::default_value();
    }
    /// If we've run out of all arguments, there is no specifier.
    static constexpr bool HasFromArgs()
    {
        return false;
    }

    /// Data that the user specified (or the default).
    data_type d_;
    /// Whether the user has specified the value or not.
    bool has_;
};

#endif // _UTILS_OPTIONALARGS_HXX_

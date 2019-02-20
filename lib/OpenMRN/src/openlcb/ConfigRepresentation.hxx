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
 * \file ConfigRepresentation.hxx
 *
 * Static representation of a config file.
 *
 * @author Balazs Racz
 * @date 31 May 2014
 */

#ifndef _OPENLCB_CONFIGREPRESENTATION_HXX_
#define _OPENLCB_CONFIGREPRESENTATION_HXX_

#include "openlcb/ConfigEntry.hxx"
#include "openlcb/MemoryConfig.hxx"

namespace openlcb
{

/// Constexpr base class for all group like structures.
class GroupBaseEntry : public openlcb::ConfigReference
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(GroupBaseEntry, ConfigReference)
    static constexpr unsigned size()
    {
        return 0;
    }
    constexpr unsigned end_offset() const
    {
        return offset_;
    }
};

/// Helper class for partial template specialization. EntryMarker<N> is used as
/// an argument to invoke specific instances of polymorphic functions, thereby
/// enabling a form of dynamically constructing a procedure from code generated
/// by macros in different places. Typical pattern is to have a function
/// foo(const EntryMarker<N>&) do something internally and then call
/// foo(EntryMarker<N-1>()) to proceed to the next item. Once all these are
/// inlined, we get a single function.
template <int N> class EntryMarker
{
public:
    constexpr EntryMarker()
    {
    }
};

/// Empty group entry that can be used for structuring the CDI configs. Does
/// not seem to be used.
class NoopGroupEntry : public ConfigReference
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(NoopGroupEntry, ConfigReference);
    constexpr unsigned end_offset() const
    {
        return offset_;
    }
    static constexpr unsigned size()
    {
        return 0;
    }
};

/// Base class for all CDI Group structures (including segment, and the whole
/// CDI entry).
class GroupBase : public openlcb::ConfigReference
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(GroupBase, ConfigReference)
    static constexpr GroupConfigOptions group_opts()
    {
        return GroupConfigOptions();
    }
    using Name = AtomConfigOptions::Name;
    using Description = AtomConfigOptions::Description;
    using MapValues = AtomConfigOptions::MapValues;
    using Min = NumericConfigOptions::Min;
    using Max = NumericConfigOptions::Max;
    using Default = NumericConfigOptions::Default;
    using Segment = GroupConfigOptions::Segment;
    using Offset = GroupConfigOptions::Offset;
    using RepName = GroupConfigOptions::RepName;
    using FixedSize = GroupConfigOptions::FixedSize;
    using Hidden = GroupConfigOptions::Hidden;
    using Manufacturer = IdentificationConfigOptions::Manufacturer;
    using Model = IdentificationConfigOptions::Model;
    using HwVersion = IdentificationConfigOptions::HwVersion;
    using SwVersion = IdentificationConfigOptions::SwVersion;
    static constexpr Segment MainCdi()
    {
        return Segment(-2);
    }
    // using MainCdi = GroupConfigOptions::MainCdi;
};

/// Helper macro for rendering code for CDI groups.
///
/// @param START_LINE the line number (in the config.hxx) file where the group
/// starts. This line number will be used to terminate the recursion looking
/// for config entries.
/// @param GroupName C++ identifier for the name of this group.
/// @param ARGS Proxied additional arguments, forwarded to the Options
/// class.
#define CDI_GROUP_HELPER(START_LINE, GroupName, ARGS...)                       \
    struct GroupName : public openlcb::GroupBase                               \
    {                                                                          \
        INHERIT_CONSTEXPR_CONSTRUCTOR(GroupName, GroupBase);                   \
        constexpr openlcb::GroupBaseEntry entry(                               \
            const openlcb::EntryMarker<START_LINE> &) const                    \
        {                                                                      \
            return openlcb::GroupBaseEntry(offset_);                           \
        }                                                                      \
        template <typename... Args>                                            \
        static constexpr openlcb::GroupConfigOptions group_opts(Args... args)  \
        {                                                                      \
            return openlcb::GroupConfigOptions(args..., ##ARGS);               \
        }                                                                      \
        static constexpr unsigned size()                                       \
        {                                                                      \
            return GroupName(0).end_offset();                                  \
        }                                                                      \
        static constexpr GroupName zero_offset_this()                          \
        {                                                                      \
            return GroupName(0);                                               \
        }                                                                      \
        template <int LINE>                                                    \
        constexpr openlcb::NoopGroupEntry entry(                               \
            const openlcb::EntryMarker<LINE> &) const                          \
        {                                                                      \
            return openlcb::NoopGroupEntry(                                    \
                entry(openlcb::EntryMarker<LINE - 1>()).end_offset());         \
        }                                                                      \
        template <int LINE>                                                    \
        static void render_content_cdi(                                        \
            const openlcb::EntryMarker<LINE> &, std::string *s)                \
        {                                                                      \
            render_content_cdi(openlcb::EntryMarker<LINE - 1>(), s);           \
        }                                                                      \
        static void render_content_cdi(                                        \
            const openlcb::EntryMarker<START_LINE> &, std::string *s)          \
        {                                                                      \
        }                                                                      \
        template <int LINE>                                                    \
        void __attribute__((always_inline))                                    \
            recursive_handle_events(const openlcb::EntryMarker<LINE> &,        \
                const openlcb::EventOffsetCallback &fn)                        \
        {                                                                      \
            recursive_handle_events(openlcb::EntryMarker<LINE - 1>(), fn);     \
        }                                                                      \
        void __attribute__((always_inline))                                    \
            recursive_handle_events(const openlcb::EntryMarker<START_LINE> &,  \
                const openlcb::EventOffsetCallback &fn)                        \
        {                                                                      \
        }                                                                      \
                                                                               \
        static constexpr openlcb::GroupConfigRenderer<GroupName>               \
        config_renderer()                                                      \
        {                                                                      \
            return openlcb::GroupConfigRenderer<GroupName>(1, GroupName(0));   \
        }

/// @todo (balazs.racz) the group config renderer should not get an instance of
/// the current group.

#define CDI_GROUP_ENTRY_HELPER(LINE, NAME, TYPE, ...)                          \
    constexpr TYPE entry(const openlcb::EntryMarker<LINE> &) const             \
    {                                                                          \
        static_assert(!group_opts().is_cdi() ||                                \
                TYPE(0).group_opts(__VA_ARGS__).is_segment(),                  \
            "May only have segments inside CDI.");                             \
        return TYPE(group_opts().is_cdi()                                      \
                ? TYPE(0).group_opts(__VA_ARGS__).get_segment_offset()         \
                : entry(openlcb::EntryMarker<LINE - 1>()).end_offset());       \
    }                                                                          \
    constexpr TYPE NAME() const                                                \
    {                                                                          \
        return entry(openlcb::EntryMarker<LINE>());                            \
    }                                                                          \
    static constexpr typename decltype(                                        \
        TYPE::config_renderer())::OptionsType NAME##_options()                 \
    {                                                                          \
        return decltype(TYPE::config_renderer())::OptionsType(__VA_ARGS__);    \
    }                                                                          \
    static void render_content_cdi(                                            \
        const openlcb::EntryMarker<LINE> &, std::string *s)                    \
    {                                                                          \
        render_content_cdi(openlcb::EntryMarker<LINE - 1>(), s);               \
        TYPE::config_renderer().render_cdi(s, ##__VA_ARGS__);                  \
    }                                                                          \
    void __attribute__((always_inline))                                        \
        recursive_handle_events(const openlcb::EntryMarker<LINE> &e,           \
            const openlcb::EventOffsetCallback &fn)                            \
    {                                                                          \
        recursive_handle_events(openlcb::EntryMarker<LINE - 1>(), fn);         \
        entry(e).handle_events(fn);                                            \
    }

/// Helper macro to generate the code needed at the end of a group.
///
/// @param LINE line number in the config.hxx where this group should end. Used
/// to start the recursion looking for group entries.
#define CDI_GROUP_END_HELPER(LINE)                                             \
    constexpr unsigned end_offset() const                                      \
    {                                                                          \
        static_assert((group_opts().fixed_size() == 0) ||                      \
                (zero_offset_this()                                            \
                              .entry(openlcb::EntryMarker<LINE>())             \
                              .end_offset() <= group_opts().fixed_size()),     \
            "FixedSize group contents too large");                             \
        return (group_opts().fixed_size() == 0)                                \
            ? entry(openlcb::EntryMarker<LINE>()).end_offset()                 \
            : offset_ + group_opts().fixed_size();                             \
    }                                                                          \
    constexpr unsigned end_buffer_length() const                               \
    {                                                                          \
        return group_opts().fixed_size() -                                     \
            (entry(openlcb::EntryMarker<LINE>()).end_offset() - offset());     \
    }                                                                          \
    static void render_content_cdi(std::string *s)                             \
    {                                                                          \
        return render_content_cdi(openlcb::EntryMarker<LINE>(), s);            \
    }                                                                          \
    void __attribute__((always_inline))                                        \
        handle_events(const openlcb::EventOffsetCallback &fn)                  \
    {                                                                          \
        recursive_handle_events(openlcb::EntryMarker<LINE>(), fn);             \
    }                                                                          \
    }

/// Starts a CDI group.
///
/// @param GroupName is the c++ name of the struct that is being defined.
/// @param ARGS are additional arguments for group options, like Name(...),
/// Description(...), Segment(...), Offset(...) or MainCdi().
#define CDI_GROUP(GroupName, ARGS...)                                          \
    CDI_GROUP_HELPER(__LINE__, GroupName, ##ARGS)

/// Adds an entry to a CDI group.
///
/// @param NAME is the c++ name of the entry
/// @param TYPE is the c++ class / struct of the entry being added
/// @param ARGS are additional arguments for the entry options, like Name(...),
/// Description(...). If a subgroup is added, then group options are also
/// allowed and they will override the respective values from the group
/// definition.
#define CDI_GROUP_ENTRY(NAME, TYPE, ARGS...)                                   \
    CDI_GROUP_ENTRY_HELPER(__LINE__, NAME, TYPE, ##ARGS)

/// Closes a CDI group structure definition.
#define CDI_GROUP_END() CDI_GROUP_END_HELPER(__LINE__)

/// Performs factory reset on a CDI variable. The variable must have a default
/// value defined. Usage:
///     CDI_FACTORY_RESET(opts_.short_retry_delay);
/// assuming that there is something like
///   CDI_GROUP_ENTRY(short_retry_delay, Uint8ConfigEntry, Default(13));
/// in the CDI group whose type opts_ is, and there is a local variable `fd` for
/// writing to the configuration file.
/// Will generate compile error if the variable does not have a default value
/// in the configuration group entry.
#define CDI_FACTORY_RESET(PATH)                                                \
    PATH().write(fd, PATH##_options().defaultvalue())

/// Requests a readout of a numeric variable with trimming. If the value
/// currently present in the config file is less than the defined minimum, then
/// sets the value to the minimum in the config file (overwriting), same for
/// max. Returns the current value after trimming.
///
/// Usage:
///   uint16_t my_value = CDI_READ_TRIMMED(cfg.seg().foo_bar, fd);
#define CDI_READ_TRIMMED(PATH, fd)                                             \
    PATH().read_or_write_trimmed(                                              \
        fd, PATH##_options().minvalue(), PATH##_options().maxvalue())

/// Defines a repeated group of a given type and a given number of repeats.
///
/// Typical usage:
///
///  using AllConsumers = RepeatedGroup<ConsumerConfig, 3>;
///
/// then add AllConsumers as an entry to the enclosing group or segment.
template <class Group, unsigned N> class RepeatedGroup : public ConfigEntryBase
{
public:
    using base_type = ConfigEntryBase;
    INHERIT_CONSTEXPR_CONSTRUCTOR(RepeatedGroup, base_type)
    static constexpr unsigned size()
    {
        return Group::size() * N;
    }
    constexpr unsigned end_offset() const
    {
        return offset() + size();
    }
    static constexpr unsigned num_repeats()
    {
        return N;
    }
    template <int K> constexpr Group entry() const
    {
        static_assert(K < N, "Tried to fetch an entry of a repeated "
                             "group that does not exist!");
        return Group(offset_ + (K * Group::size()));
    }

    Group entry(unsigned k)
    {
        HASSERT(k < N);
        return Group(offset_ + (k * Group::size()));
    }

    static constexpr GroupConfigRenderer<Group> config_renderer()
    {
        /// @todo (balazs.racz) get rid of the instance of Group here.
        return GroupConfigRenderer<Group>(N, Group(0));
    }

    void handle_events(const EventOffsetCallback &fn)
    {
        for (unsigned i = 0; i < N; ++i)
        {
            entry(i).handle_events(fn);
        }
    }
};

///
/// Defines an empty group with no members, but blocking a certain amount of
/// space in the rendered configuration.
///
template <unsigned N> class EmptyGroup : public ConfigEntryBase
{
public:
    using base_type = ConfigEntryBase;
    INHERIT_CONSTEXPR_CONSTRUCTOR(EmptyGroup, base_type)
    static constexpr unsigned size()
    {
        return N;
    }
    constexpr unsigned end_offset() const
    {
        return offset() + size();
    }
    static constexpr EmptyGroupConfigRenderer config_renderer()
    {
        return EmptyGroupConfigRenderer(N);
    }
};

/// Base class for all entries that can appear in the MainCdi group. THe common
/// property of these entries is that they do not rely on the offset/size
/// propagation of the previous entries, because they either do not take part
/// in the layout algorithm (e.g. the <identification> tag) or they specify the
/// origin explcitly.
class ToplevelEntryBase : public ConfigEntryBase
{
public:
    using base_type = ConfigEntryBase;
    INHERIT_CONSTEXPR_CONSTRUCTOR(ToplevelEntryBase, base_type)
    template<typename... Args>
    static constexpr GroupConfigOptions group_opts(Args... args)
    {
        return GroupConfigOptions(GroupConfigOptions::Segment(1000));
    }
    static constexpr unsigned size()
    {
        return 0;
    }
    constexpr unsigned end_offset() const
    {
        return offset() + size();
    }
};

/// Add this entry to the beginning of the CDI group to render an
/// "<identification>" tag at the beginning of the output cdi.xml. Requires a
/// global symbol of @ref openlcb::SNIP_STATIC_DATA to fill in the specific
/// values of the identification tree.
class Identification : public ToplevelEntryBase
{
public:
    using base_type = ToplevelEntryBase;
    INHERIT_CONSTEXPR_CONSTRUCTOR(Identification, base_type)
    static constexpr IdentificationRenderer config_renderer()
    {
        return IdentificationRenderer();
    }
};

/// Renders an "<acdi>" tag in the CDI group.
class Acdi : public ToplevelEntryBase
{
public:
    using base_type = ToplevelEntryBase;
    INHERIT_CONSTEXPR_CONSTRUCTOR(Acdi, base_type)
    static constexpr AcdiRenderer config_renderer()
    {
        return AcdiRenderer();
    }
};

/// Configuration description for a segment containing the ACDI user-modifiable
/// data. The implementation refers to the ACDI-userdata space number and does
/// not depend on where the actual data is located.
CDI_GROUP(
    UserInfoSegment, Segment(MemoryConfigDefs::SPACE_ACDI_USR), Offset(1));
/// User name entry
CDI_GROUP_ENTRY(name, StringConfigEntry<63>, //
    Name("User name"),                       //
    Description(
        "This name will appear in network browsers for the current node."));
/// User description entry
CDI_GROUP_ENTRY(description, StringConfigEntry<64>, //
    Name("User description"),                       //
    Description("This description will appear in network browsers for the "
                "current node."));
/// Signals termination of the group.
CDI_GROUP_END();

/// Configuration description for internal configuration variables. This should
/// preferably not be user-visible in the CDI, but the space has to be reserved
/// in the configuration EEPROM.
CDI_GROUP(InternalConfigData, Name("Internal data"),
    Description("Do not change these settings."));
/// Used to detect firmwares that have their config layout set in incompatible
/// ways.
CDI_GROUP_ENTRY(version, Uint16ConfigEntry, Name("Version"));
/// Last two bytes ofthe next available event ID that can be assigned in a
/// factory reset to the producers/consumers.
CDI_GROUP_ENTRY(next_event, Uint16ConfigEntry, Name("Next event ID"));
CDI_GROUP_END();

} // namespace openlcb

/// Helper function defined in CompileCdiMain.cxx.
template <typename CdiType>
void render_cdi_helper(const CdiType &t, string ns, string name);

template <int N> class CdiRenderHelper;

/// Forward declaration of the recursive helper template for adding multiple
/// CDIs to a single binary image.
template <int N> void render_all_cdi();

/// End-of-recursion template instantiation for CDI rendering.
template <> inline void render_all_cdi<0>()
{
}

/** Use this macro if additional CDI entries need to be rendered, in addition
 * to the openlcb::ConfigDef. Example usage:
 *
 * } // namespace XXX -- RENDER_CDI will work only if at toplevel!
 *
 * RENDER_CDI(openlcb, ConfigDef, "CDI", 3);
 *   this will create CDI_DATA and CDI_SIZE symbols.
 *
 * @param NS is the namespace without quotes
 * @param TYPE is the typename of the CDI root group (with MainCdi())
 * @param NAME is the basenamefor the output symbols. Generated will be
 *    $(NAME)_DATA and $(NAME)_SIZE
 * @param N is a unique integer between 2 and 10 for the invocation.
 */
#define RENDER_CDI(NS, TYPE, NAME, N)                                          \
    template <> inline void render_all_cdi<N>()                                \
    {                                                                          \
        NS::TYPE def(0);                                                       \
        render_cdi_helper(def, #NS, NAME);                                     \
        render_all_cdi<N - 1>();                                               \
    }

#endif // _OPENLCB_CONFIGREPRESENTATION_HXX_

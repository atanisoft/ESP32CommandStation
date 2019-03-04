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
 * \file ConfigRenderer.hxx
 *
 * Helper classes for creating a CDI xml file.
 *
 * @author Balazs Racz
 * @date 6 June 2015
 */

#ifndef _OPENLCB_CONFIGRENDERER_HXX_
#define _OPENLCB_CONFIGRENDERER_HXX_

#include <climits>
#include <string>

#include "openlcb/SimpleNodeInfo.hxx"
#include "utils/OptionalArgs.hxx"
#include "utils/StringPrintf.hxx"

namespace openlcb
{

/// Configuration options for rendering CDI (atom) data elements.
struct AtomConfigDefs
{
    DECLARE_OPTIONALARG(Name, name, const char *, 0, nullptr);
    DECLARE_OPTIONALARG(Description, description, const char *, 1, nullptr);
    DECLARE_OPTIONALARG(MapValues, mapvalues, const char *, 2, nullptr);
    using Base = OptionalArg<AtomConfigDefs, Name, Description, MapValues>;
};

/// Configuration implementation class for CDI Atom elements (strings, events
/// and numbers).
class AtomConfigOptions : public AtomConfigDefs::Base
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(AtomConfigOptions, AtomConfigDefs::Base);

    /// Represent the value enclosed in the "<name>" tag of the data element.
    DEFINE_OPTIONALARG(Name, name, const char *);
    /// Represent the value enclosed in the "<description>" tag of the data
    /// element.
    DEFINE_OPTIONALARG(Description, description, const char *);
    /// Represent the value enclosed in the "<map>" tag of the data element.
    DEFINE_OPTIONALARG(MapValues, mapvalues, const char *);

    void render_cdi(std::string *r) const
    {
        if (name())
        {
            *r += StringPrintf("<name>%s</name>\n", name());
        }
        if (description())
        {
            *r +=
                StringPrintf("<description>%s</description>\n", description());
        }
        if (mapvalues())
        {
            *r += StringPrintf("<map>%s</map>\n", mapvalues());
        }
    }
};

/// Helper class for rendering an atom data element into the cdi.xml.
class AtomConfigRenderer
{
public:
    enum
    {
        SKIP_SIZE = 0xffffffff,
    };

    typedef AtomConfigOptions OptionsType;

    constexpr AtomConfigRenderer(const char *tag, unsigned size)
        : tag_(tag)
        , size_(size)
    {
    }

    template <typename... Args> void render_cdi(string *s, Args... args) const
    {
        *s += StringPrintf("<%s", tag_);
        if (size_ != SKIP_SIZE)
        {
            *s += StringPrintf(" size=\'%u\'", size_);
        }
        *s += ">\n";
        AtomConfigOptions(args...).render_cdi(s);
        *s += StringPrintf("</%s>\n", tag_);
    }

private:
    /// XML tag for this atom.
    const char *tag_;
    /// The size attribute of the configuration atom.
    unsigned size_;
};

/// Declarations for the options for numeric CDI entries.
struct NumericConfigDefs : public AtomConfigDefs
{
    // This is needed for inheriting declarations.
    using AtomConfigDefs::check_arguments_are_valid;
    DECLARE_OPTIONALARG(Min, minvalue, int, 6, INT_MAX);
    DECLARE_OPTIONALARG(Max, maxvalue, int, 7, INT_MAX);
    DECLARE_OPTIONALARG(Default, defaultvalue, int, 8, INT_MAX);
    using Base = OptionalArg<NumericConfigDefs, Name, Description, MapValues,
        Min, Max, Default>;
};

/// Definitions for the options for numeric CDI entries.
class NumericConfigOptions : public NumericConfigDefs::Base
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(
        NumericConfigOptions, NumericConfigDefs::Base);

    /// Represent the value enclosed in the "<name>" tag of the data element.
    DEFINE_OPTIONALARG(Name, name, const char *);
    /// Represent the value enclosed in the <description> tag of the data
    /// element.
    DEFINE_OPTIONALARG(Description, description, const char *);
    /// Represent the value enclosed in the <map> tag of the data element.
    DEFINE_OPTIONALARG(MapValues, mapvalues, const char *);
    DEFINE_OPTIONALARG(Min, minvalue, int);
    DEFINE_OPTIONALARG(Max, maxvalue, int);
    DEFINE_OPTIONALARG(Default, defaultvalue, int);

    void render_cdi(std::string *r) const
    {
        if (name())
        {
            *r += StringPrintf("<name>%s</name>\n", name());
        }
        if (description())
        {
            *r +=
                StringPrintf("<description>%s</description>\n", description());
        }
        if (minvalue() != INT_MAX)
        {
            *r += StringPrintf("<min>%d</min>\n", minvalue());
        }
        if (maxvalue() != INT_MAX)
        {
            *r += StringPrintf("<max>%d</max>\n", maxvalue());
        }
        if (defaultvalue() != INT_MAX)
        {
            *r += StringPrintf("<default>%d</default>\n", defaultvalue());
        }
        if (mapvalues())
        {
            *r += StringPrintf("<map>%s</map>\n", mapvalues());
        }
    }

    int clip(int value) {
        if (has_minvalue() && (value < minvalue())) {
            value = minvalue();
        }
        if (has_maxvalue() && (value > maxvalue())) {
            value = maxvalue();
        }
        return value;
    }
};

/// Helper class for rendering a numeric data element into the cdi.xml.
class NumericConfigRenderer
{
public:
    enum
    {
        SKIP_SIZE = 0xffffffff,
    };

    typedef NumericConfigOptions OptionsType;

    constexpr NumericConfigRenderer(const char *tag, unsigned size)
        : tag_(tag)
        , size_(size)
    {
    }

    template <typename... Args> void render_cdi(string *s, Args... args) const
    {
        *s += StringPrintf("<%s", tag_);
        if (size_ != SKIP_SIZE)
        {
            *s += StringPrintf(" size=\'%u\'", size_);
        }
        *s += ">\n";
        NumericConfigOptions(args...).render_cdi(s);
        *s += StringPrintf("</%s>\n", tag_);
    }

private:
    /// XML tag for this atom.
    const char *tag_;
    /// The size attribute of the configuration atom.
    unsigned size_;
};

/// Configuration options for the CDI group element, as well as representing
/// and distinguishing alternate uses of the BEGIN_GROUP/EXTEND_GROUP/END_GROUP
/// syntax, such as for the toplevel CDI node and for representing segments..
struct GroupConfigDefs : public AtomConfigDefs
{
    // This is needed for inheriting declarations.
    using AtomConfigDefs::check_arguments_are_valid;
    DECLARE_OPTIONALARG(Offset, offset, int, 10, INT_MAX);
    DECLARE_OPTIONALARG(Segment, segment, int, 11, -1);
    DECLARE_OPTIONALARG(RepName, repname, const char*, 12, nullptr);
    DECLARE_OPTIONALARG(FixedSize, fixed_size, unsigned, 13, 0);
    DECLARE_OPTIONALARG(Hidden, hidden, int, 14, 0);
    using Base = OptionalArg<GroupConfigDefs, Name, Description, Segment,
        Offset, RepName, FixedSize, Hidden>;
};

/// Implementation class for the condifuration options of a CDI group element.
class GroupConfigOptions : public GroupConfigDefs::Base
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(GroupConfigOptions, GroupConfigDefs::Base);

    DEFINE_OPTIONALARG(Name, name, const char *);
    /// Represent the value enclosed in the <description> tag of the data
    /// element.
    DEFINE_OPTIONALARG(Description, description, const char *);

    /// Represents the 'offset' attribute for groups and the 'origin' attribute
    /// for segments.
    DEFINE_OPTIONALARG(Offset, offset, int);

    /// Declares that the group is a segment (and thus may be used in the
    /// toplevel CDI.
    DEFINE_OPTIONALARG(Segment, segment, int);

    /// Specifies for the UI what the repetitions of this group should be
    /// called.
    DEFINE_OPTIONALARG(RepName, repname, const char*);

    /// Specifies that the size of this group should be fixed to this many
    /// bytes, even if the contents are smaller. Creates a compile error if the
    /// size is bigger.
    DEFINE_OPTIONALARG(FixedSize, fixed_size, int);

    /// If non-zero, the group contents will not be rendered in the CDI,
    /// effectively hiding hte settings from the user. The space will still be
    /// reserved and skipped.
    DEFINE_OPTIONALARG(Hidden, hidden, int);

    /// Declares that this group is a toplevel CDI. Causes the group to render
    /// the xml header.
    /// static constexpr Segment MainCdi() { return Segment(-2); }
    /*struct MainCdi : public Segment
    {
        constexpr MainCdi()
            : Segment(-2)
        {
        }
        };*/

    ///
    /// @return true if this group is a toplevel CDI definition and shall only
    /// allow segments and other toplevel-compatible entries (but no data
    /// elements).
    ///
    constexpr bool is_cdi() const
    {
        return segment() == -2;
    }

    ///
    /// @return true if this group is a segment definition.
    ///
    constexpr bool is_segment() const
    {
        return segment() >= 0;
    }

    ///
    /// @return the origin of the current segment or zero (default) if not
    /// specified.
    ///
    constexpr unsigned get_segment_offset() const
    {
        return offset() == INT_MAX ? 0 : offset();
    }

    void render_cdi(std::string *r) const
    {
        if (name())
        {
            *r += StringPrintf("<name>%s</name>\n", name());
        }
        if (description())
        {
            *r +=
                StringPrintf("<description>%s</description>\n", description());
        }
        if (repname())
        {
            *r +=
                StringPrintf("<repname>%s</repname>\n", repname());
        }
    }
};

/// Helper class for rendering an empty group of a given size into the cdi.xml.
class EmptyGroupConfigRenderer
{
public:
    enum
    {
        SKIP_SIZE = 0xffffffff,
    };

    typedef GroupConfigOptions OptionsType;

    constexpr EmptyGroupConfigRenderer(unsigned size)
        : size_(size)
    {
    }

    template <typename... Args> void render_cdi(string *s, Args... args) const
    {
        *s += StringPrintf("<group offset='%u'/>\n", size_);
    }

private:
    /// The number of bytes this group has to skip.
    unsigned size_;
};

/// Helper class for rendering the cdi.xml of groups, segments and the toplevel
/// CDI node.
template <class Body> class GroupConfigRenderer
{

public:
    typedef GroupConfigOptions OptionsType;

    constexpr GroupConfigRenderer(unsigned replication, Body body)
        : replication_(replication)
        , body_(body)
    {
    }

    template <typename... Args> void render_cdi(string *s, Args... args)
    {
        GroupConfigOptions opts(args..., Body::group_opts());
        if (opts.hidden())
        {
            EmptyGroupConfigRenderer(Body::size()).render_cdi(s);
            return;
        }
        const char *tag = nullptr;
        *s += "<";
        if (opts.is_cdi())
        {
            *s += "?xml version=\"1.0\"?>\n<";
            tag = "cdi";
            *s += tag;
            *s += " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
                  "xsi:noNamespaceSchemaLocation=\"http://openlcb.org/schema/"
                  "cdi/1/1/cdi.xsd\"";
            HASSERT(replication_ == 1);
            HASSERT(opts.name() == nullptr && opts.description() == nullptr);
        }
        else if (opts.segment() == -1)
        {
            // Regular group
            tag = "group";
            *s += tag;
            if (replication_ != 1)
            {
                *s += StringPrintf(" replication='%u'", replication_);
            }
        }
        else
        {
            // Segment inside CDI.
            tag = "segment";
            *s += tag;
            *s += StringPrintf(" space='%d'", opts.segment());
            if (opts.get_segment_offset() != 0)
            {
                *s += StringPrintf(" origin='%d'", opts.get_segment_offset());
            }
            HASSERT(replication_ == 1);
        }
        *s += ">\n";
        opts.render_cdi(s);
        body_.render_content_cdi(s);
        if (opts.fixed_size() && (body_.end_buffer_length() > 0)) {
            *s += StringPrintf(
                "<group offset='%u'/>\n", body_.end_buffer_length());
        }
        *s += StringPrintf("</%s>\n", tag);
    }

private:
    /// For regular groups, the count of replicas.
    unsigned replication_;
    /// Object representing the contents of this group. Must have a
    /// render_content_cdi() call.
    Body body_;
};

extern const SimpleNodeStaticValues SNIP_STATIC_DATA;

/// Configuration options for rendering CDI (identification) data elements.
struct IdentificationConfigDefs
{
    DECLARE_OPTIONALARG(Manufacturer, manufacturer, const char *, 0, nullptr);
    DECLARE_OPTIONALARG(Model, model, const char *, 1, nullptr);
    DECLARE_OPTIONALARG(HwVersion, hardware_version, const char *, 2, nullptr);
    DECLARE_OPTIONALARG(SwVersion, software_version, const char *, 3, nullptr);
    using Base = OptionalArg<IdentificationConfigDefs, Manufacturer, Model,
        HwVersion, SwVersion>;
};

/// Configuration implementation options for rendering CDI (identification)
/// data elements.
class IdentificationConfigOptions : public IdentificationConfigDefs::Base
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(
        IdentificationConfigOptions, IdentificationConfigDefs::Base);

    DEFINE_OPTIONALARG(Manufacturer, manufacturer, const char *);
    DEFINE_OPTIONALARG(Model, model, const char *);
    DEFINE_OPTIONALARG(HwVersion, hardware_version, const char *);
    DEFINE_OPTIONALARG(SwVersion, software_version, const char *);
};

/// Helper class for rendering the "<identification>" tag.
class IdentificationRenderer
{
public:
    typedef IdentificationConfigOptions OptionsType;

    constexpr IdentificationRenderer()
    {
    }

    static void render_tag(const char *tag, const char *value, string *s)
    {
        *s += StringPrintf("<%s>%s</%s>\n", tag, value, tag);
    }

    static const char *alt(const char *opt, const char *def)
    {
        if (opt)
            return opt;
        return def;
    }

    template <typename... Args> void render_cdi(string *s, Args... args) const
    {
        IdentificationConfigOptions opts(args...);
        extern const SimpleNodeStaticValues SNIP_STATIC_DATA;
        *s += "<identification>\n";
        render_tag("manufacturer",
            alt(opts.manufacturer(), SNIP_STATIC_DATA.manufacturer_name), s);
        render_tag("model", alt(opts.model(), SNIP_STATIC_DATA.model_name), s);
        render_tag("hardwareVersion",
            alt(opts.hardware_version(), SNIP_STATIC_DATA.hardware_version), s);
        render_tag("softwareVersion",
            alt(opts.software_version(), SNIP_STATIC_DATA.software_version), s);
        *s += "</identification>\n";
    }
};

/// Helper class for rendering the "<acdi>" tag.
class AcdiRenderer
{
public:
    constexpr AcdiRenderer()
    {
    }

    typedef AtomConfigOptions OptionsType;

    void render_cdi(string *s) const
    {
        s->append("<acdi/>\n");
    }
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGRENDERER_HXX_

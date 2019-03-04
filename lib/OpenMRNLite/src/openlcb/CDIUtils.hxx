/** \copyright
 * Copyright (c) 2017, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file CDIUtils.hxx
 *
 * Utility library for interpreting CDI XML files.
 *
 * @author Balazs Racz
 * @date 31 Aug 2017
 */

#ifndef _OPENLCB_CDIUTILS_HXX_
#define _OPENLCB_CDIUTILS_HXX_

#include "sxmlc.h"

namespace openlcb
{

class CDIUtils
{
public:
    /// Helper class for unique_ptr to delete an XML document correctly.
    struct XMLDocDeleter
    {
        /// Deletion operator. @param doc the object to be freed.
        void operator()(XMLDoc *doc)
        {
            if (!doc)
                return;
            cleanup_doc(doc);
            XMLDoc_free(doc);
            delete doc;
        }
    };

    /// Smart pointer class for holding XML documents.
    typedef std::unique_ptr<XMLDoc, XMLDocDeleter> xmldoc_ptr_t;

    /// Helper class for deleting sxml strings.
    struct SXmlStringDeleter
    {
        /// Deletion operator. @param data the SXML string to be freed.
        void operator()(const SXML_CHAR *data)
        {
            free(const_cast<SXML_CHAR *>(data));
        }
    };
    /// Smart pointer class for holding C strings from sxml. Appropriately
    /// frees the string upon going out of scope.
    typedef std::unique_ptr<const SXML_CHAR, SXmlStringDeleter> xmlstring_t;

    /// Searches the list of children for the first child with a specific tag.
    /// @param parent is the node whose children to search.
    /// @param tag is which child to look for.
    /// @return nullptr if not found, else the first occurrence of tag in the
    /// children.
    static XMLNode *find_child_or_null(const XMLNode *parent, const char *tag)
    {
        auto count = XMLNode_get_children_count(parent);
        for (int i = 0; i < count; ++i)
        {
            auto *c = XMLNode_get_child(parent, i);
            if (strcmp(tag, c->tag) == 0)
                return c;
        }
        return nullptr;
    }

    /// Finds the name of a CDI element.
    ///
    /// @param node is a CDI element (segment, group, or config element).
    /// @param def is the default to return if no <name> element exists or if
    /// it has no text.
    /// @return finds a child tag <name> and returns its contents. Returns
    /// empty string if no <name> was found.
    static string find_node_name(const XMLNode *node, const char *def)
    {
        auto *n = find_child_or_null(node, "name");
        if (n == nullptr || n->text == nullptr)
            return def;
        return n->text;
    }

    /// Find the description of a CDI element.
    ///
    /// @param node is a CDI element (segment, group, or config element).
    /// @return finds a child tag <description> and returns its
    /// contents. Returns empty string if no <description> was found.
    static string find_node_description(const XMLNode *node)
    {
        auto *n = find_child_or_null(node, "description");
        if (n == nullptr || n->text == nullptr)
            return "";
        return n->text;
    }

    // this is a helper function that checks to see if the passed node has an 
    // element named by *tag in it. Sets the bool found to true or false
    // accordingly and also returns a nullptr (if element is undefined)
    // otherwise it will return the value of the element
    static int find_numeric_element(
        const XMLNode *parent, const char *tag, bool *found)
    {
        auto *n = find_child_or_null(parent, tag);
        // if there is no element defined set bool to false, return any value
        if (n == nullptr || n->text == nullptr){
            *found = false;
            return 0;// there is no defined element
        }
        // if the numeric element was found set the bool to true and return it
        *found = true;
        return atoi(n->text); // convert to int and return element value
    }

    /// Finds the min value of the xml integer, if there isn't one it returns
    /// a null pointer
    /// @param "node" is a CDI element (segment, group, or config element).
    /// @param "found" will be set to true if there was a minimum found, false if 
    /// the XML did not contain the respective tag.
    /// @return contents of the <min> tag, unspecified if !found. 
    static int find_node_min(const XMLNode *node, bool *found)
    {
        return find_numeric_element(node, "min", found);
    }

    /// Finds the max value of the xml integer, if there isn't one it returns
    /// a null pointer
    /// @param "node" is a CDI element (segment, group, or config element).
    /// @param "found" will be set to true if there was a maximum found, false if 
    /// the XML did not contain the respective tag.
    /// @return contents of the <max> tag, unspecified if !found. 
    static int find_node_max(const XMLNode *node, bool *found)
    {
        return find_numeric_element(node, "max", found);
    }

    /// Finds the default value of the xml integer, if there isn't one it returns
    /// a null pointer
    /// @param "node" is a CDI element (segment, group, or config element).
    /// @param "found" will be set to true if there was a defaultimum found, false if 
    /// the XML did not contain the respective tag.
    /// @return contents of the <default> tag, unspecified if !found. 
    static int find_node_default(const XMLNode *node, bool *found)
    {
        return find_numeric_element(node, "default", found);
    }
    
    /// Clears out all user info structure pointers. This is necessary to use
    /// the new_userinfo call below. Call this after the XML has been
    /// successfully parsed.
    /// @param doc document to prepare up.
    static void prepare_doc(XMLDoc *doc)
    {
        auto *node = XMLDoc_root(doc);
        while (node)
        {
            node->user = nullptr;
            node = XMLNode_next(node);
        }
    }

    /// Deletes all userinfo structures allocated in a doc.
    /// @param doc document to clean up.
    static void cleanup_doc(XMLDoc *doc)
    {
        auto *node = XMLDoc_root(doc);
        while (node)
        {
            free(node->user);
            node->user = nullptr;
            node = XMLNode_next(node);
        }
    }

    /// Allocates a new object of type T for the node as userinfo; calls T's
    /// constructor with args..., stores the resulting pointer in the userinfo
    /// pointer of node.
    /// @param info output argument for the userinfo structure pointer.
    /// @param node which XML node the userinfo should point at.
    /// @param args... forwarded as the constructor arguments for T (can be
    /// empty).
    template <class T, typename... Args>
    static void new_userinfo(T **info, XMLNode *node, Args &&... args)
    {
        HASSERT(node->user == nullptr);
        static_assert(std::is_trivially_destructible<T>::value == true,
            "Userdata attached to nodes must be trivially destructible");
        *info = static_cast<T *>(malloc(sizeof(T)));
        new (*info) T(std::forward<Args>(args)...);
        node->user = *info;
    }

    /// Retrieve the userinfo structure from an XML node.
    /// @param info will be set to the userinfo structure using an unchecked
    /// cast to T. This variable must be of the same (or compatible) type as
    /// what the userinfo has been allocated to.
    /// @param node is the XML element node whose userinfo we are trying to
    /// fetch
    template <class T> static void get_userinfo(T **info, const XMLNode *node)
    {
        HASSERT(node);
        *info = static_cast<T *>(node->user);
    }

    /// Allocation data we hold about a Data Element in its userinfo structure.
    struct NodeInfo
    {
        /// Offset of the address of this element from the address of the
        /// parent group element. This is the sum of size values of the
        /// preceding elements within the given group. Inside a repeated group
        /// these offsets are counted from the current repetition start offset.
        int offset_from_parent = 0;
        /// Total number of bytes that this element occupies. This includes all
        /// repetitions for a repeated group.
        int size = 0;
    };

    /// Helper function to find and convert an attribute to a number.
    /// @param node is the XML node of the element whose attribute we're looking
    /// for
    /// @param attr_name is a C-string for the attribute name we're looking for
    /// @param def is the default value that will be returned if the attribute
    /// is not found.
    /// @return attribute value, or `def` if not found, or zero if the
    /// attribute is found but the value is not convertible to an integer.
    static int get_numeric_attribute(
        const XMLNode *node, const char *attr_name, int def = 0)
    {
        const SXML_CHAR *attr_value;
        XMLNode_get_attribute_with_default(
            const_cast<XMLNode *>(node), attr_name, &attr_value, nullptr);
        xmlstring_t d(attr_value);
        if ((!attr_value) || (attr_value[0] == 0))
        {
            return def;
        }
        return atoi(attr_value);
    }

    /// Used to classify elements.
    enum class DataType {
        UNKNOWN = 0,
        GROUP,
        INT,
        FLOAT,
        STRING,
        EVENTID
    };

    /// Classifies XML elements to node types.
    /// @param child is an XML element under a group.
    /// @return node type or UNKNOWN
    static DataType get_type_from_node(XMLNode *child)
    {
        if (strcmp(child->tag, "group") == 0)
        {
            return DataType::GROUP;
        }
        if (strcmp(child->tag, "int") == 0)
        {
            return DataType::INT;
        }
        if (strcmp(child->tag, "eventid") == 0)
        {
            return DataType::EVENTID;
        }
        if (strcmp(child->tag, "string") == 0)
        {
            return DataType::STRING;
        }
        if (strcmp(child->tag, "float") == 0)
        {
            return DataType::FLOAT;
        }
        return DataType::UNKNOWN;
    };

    /// Allocates all userinfo structures within a segment and performs the
    /// offset layout algorithm.
    /// @param segment is the XML node of the <segment> element.
    static void layout_segment(XMLNode *segment)
    {
        HASSERT(strcmp(segment->tag, "segment") == 0);
        unsigned current_offset = get_numeric_attribute(segment, "origin");
        NodeInfo *info;
        new_userinfo(&info, segment);
        info->offset_from_parent = current_offset;
        if (XMLNode_get_children_count(segment) == 0)
            return;
        XMLNode *current_parent = segment;
        XMLNode *current_child = XMLNode_get_child(segment, 0);
        NodeInfo *parent_info = info;
        while (true)
        {
            if (strcmp(current_child->tag, "name") == 0 ||
                strcmp(current_child->tag, "description") == 0 ||
                strcmp(current_child->tag, "repname") == 0)
            {
                // Do nothing, not a data element
            }
            else
            {
                new_userinfo(&info, current_child);
                parent_info->size +=
                    get_numeric_attribute(current_child, "offset", 0);
                info->offset_from_parent = parent_info->size;
                auto type = get_type_from_node(current_child);
                switch (type)
                {
                    case DataType::UNKNOWN:
                        // Probably should not get here.
                        break;
                    case DataType::EVENTID:
                        info->size = 8;
                        break;
                    case DataType::STRING:
                    case DataType::INT:
                    case DataType::FLOAT:
                        info->size =
                            get_numeric_attribute(current_child, "size", 1);
                        break;
                    case DataType::GROUP:
                        if (XMLNode_get_children_count(current_child) > 0)
                        {
                            current_parent = current_child;
                            current_child =
                                XMLNode_get_child(current_parent, 0);
                            get_userinfo(&parent_info, current_parent);
                            continue;
                        }
                        // an empty group has size == 0 and we don't need to do
                        // anything here.
                        break;
                }
                parent_info->size += info->size;
            }

            // Move to next child.
            while ((current_child = XMLNode_next_sibling(current_child)) ==
                nullptr)
            {
                // End of children; must go up.
                if (current_parent == segment)
                {
                    // nowhere to go up
                    break;
                }
                current_child = current_parent;
                current_parent = current_child->father;
                // handle groups with repetitions
                get_userinfo(&info, current_child);
                get_userinfo(&parent_info, current_parent);
                int repcount = get_replication(current_child);
                info->size *= repcount;
                parent_info->size += info->size;
            }
            if (current_parent == segment && current_child == nullptr)
            {
                // end of iteration
                break;
            }
        }
    }

    /// @return the number of replicas a group has, if it is a repeated group,
    /// or 1 if it is a non-repeated group.
    /// @param group is the XML node of the <group> element to query.
    static int get_replication(const XMLNode *group)
    {
        HASSERT(strcmp(group->tag, "group") == 0);
        return get_numeric_attribute(group, "replication", 1);
    }

    struct CDINodeRep
    {
        /// Element in the XML where we are. This is a segment or a group node.
        const XMLNode *node_;
        /// address in the current space of the beginning of the current
        /// node. For segments, this is the origin; for groups this is the
        /// address of the virtual zero-length data element at the beginning of
        /// the group. For repeated groups this is meaningful only with a given
        /// repetition of a group.
        unsigned address_;

        /// Default constructor. Probably should not be used; maybe only for
        /// the root of the CDI which is not inside any segment.
        CDINodeRep()
            : node_(nullptr)
            , address_(0)
        {
        }

        /// Initializes a group rep from an arbitrary XML element (e.g. the cdi
        /// root). Tihs does not allow child computations.
        /// @param node is the XML element.
        CDINodeRep(const XMLNode *node, std::nullptr_t)
            : node_(node)
            , address_(0)
        {
        }

        /// Initializes a group rep from a segment root.
        /// @param segment is the XML element representing the segment root.
        CDINodeRep(const XMLNode *segment)
        {
            NodeInfo *info;
            get_userinfo(&info, segment);
            HASSERT(info);
            node_ = segment;
            address_ = info->offset_from_parent;
        }

        /// Initializes a group rep which has no replication or an entry rep.
        /// @param parent is the Node representation of the parent group or
        /// segment.
        /// @param child is the XML element for the current (child). If it is a
        /// group, it must have no replication.
        CDINodeRep(const CDINodeRep *parent, const XMLNode *child)
        {
            HASSERT(child->father == parent->node_);
            node_ = child;
            if (strcmp(child->tag, "group") == 0)
            {
                HASSERT(get_replication(child) == 1);
            }
            address_ = parent->get_child_address(child);
        }

        /// Initializes a group rep for a given replica.
        /// @param parent is the Node representation of the parent group or
        /// segment.
        /// @param group is the XML element for the current (child) group. it
        /// must have replication.
        /// @param replica is the number of the replication (zero to
        /// replication - 1).
        CDINodeRep(const CDINodeRep *parent, const XMLNode *group, unsigned replica)
        {
            HASSERT(group->father == parent->node_);
            node_ = group;
            int replication = get_replication(group);
            HASSERT(replication > 1);
            NodeInfo *info;
            get_userinfo(&info, group);
            HASSERT(info);
            unsigned base_address = parent->get_child_address(group);
            unsigned stride = info->size / replication;
            address_ = base_address + stride * replica;
        }

        /// Resets the current representation. Use any constructor argument
        /// set.
        template <typename... Args> void reset(Args &&... args)
        {
            new (this) CDINodeRep(std::forward<Args>(args)...);
        }

        /// Gets the absolute address of a given child in the current segment.
        /// @param child an element which is a child of node_
        /// @return the absolute address within the CDI segment of child.
        unsigned get_child_address(const XMLNode *child) const
        {
            HASSERT(child->father == node_);
            NodeInfo *info;
            get_userinfo(&info, child);
            HASSERT(info);
            return address_ + info->offset_from_parent;
        }

        /// Gets the size of a child (number of bytes occupied).
        unsigned get_child_size(const XMLNode *child) const
        {
            HASSERT(child->father == node_);
            NodeInfo *info;
            get_userinfo(&info, child);
            HASSERT(info);
            return info->size;
        }
    };

private:
    /// Static class; never instantiated.
    CDIUtils();
};

} // namespace openlcb

#endif // _OPENLCB_CDIUTILS_HXX_

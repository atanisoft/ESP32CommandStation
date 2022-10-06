/*
 * SPDX-FileCopyrightText: 2015 Balazs Racz
 * SPDX-FileCopyrightText: 2020-2022 Mike Dunston (atanisoft)
 *
 * SPDX-License-Identifier: MIT
 */

#include "XmlGenerator.hxx"
#include <utils/format_utils.hxx>

namespace trainmanager
{

ssize_t XmlGenerator::read(size_t offset, void* buf, size_t len)
{
  if (offset < fileOffset_)
  {
    return -1;
  }
  offset -= fileOffset_;
  char* output = static_cast<char*>(buf);

  while (len > 0)
  {
    if (pendingActions_.empty())
    {
      generate_more();
      if (pendingActions_.empty())
      {
        // EOF.
        break;
      }
      TypedQueue<GeneratorAction> reversed;
      while (!pendingActions_.empty())
      {
        reversed.push_front(pendingActions_.pop_front());
      }
      std::swap(pendingActions_, reversed);
      init_front_action();
    }

    const char* b = get_front_buffer();
    bufferOffset_ = 0;
    /*if (offset <= bufferOffset_) {
      bufferOffset_ = offset;
      offset = 0;
      }*/
    // Skip data that we don't need.
    while (*b && offset > 0)
    {
      --offset;
      ++b;
      ++bufferOffset_;
    }
    // Copy data from the front action buffer.
    while (*b && len > 0)
    {
      *output++ = *b++;
      len--;
      bufferOffset_++;
    }
    if (!*b)
    {
      // Consume front of the actions.
      delete pendingActions_.pop_front();
      fileOffset_ += bufferOffset_;
      if (!pendingActions_.empty()) {
        init_front_action();
      }
    }
  }
  return output - static_cast<char*>(buf);
}

const char* XmlGenerator::get_front_buffer()
{
  switch (pendingActions_.front()->type)
  {
    case RENDER_INT:
    {
      return buffer_;
    }
    case CONST_LITERAL:
    {
      return static_cast<const char*>(pendingActions_.front()->pointer);
    }
    default:
      DIE("Unknown XML generation action.");
  }
}

void XmlGenerator::init_front_action()
{
  bufferOffset_ = 0;
  switch (pendingActions_.front()->type)
  {
    case RENDER_INT:
    {
      integer_to_buffer(pendingActions_.front()->integer, buffer_);
      break;
    }
    case CONST_LITERAL:
    {
      break;
    }
    default:
      DIE("Unknown XML generation action.");
  }
}

void XmlGenerator::internal_reset()
{
  fileOffset_ = 0;
  while (!pendingActions_.empty())
  {
    delete pendingActions_.pop_front();
  }
}

}  // namespace trainmanager
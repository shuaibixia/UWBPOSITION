
#ifndef __log_h__
#define __log_h__

#include <sstream>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/channel_logger.hpp>

#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/core/null_deleter.hpp>

namespace logging   = boost::log;
namespace src       = boost::log::sources;
namespace expr      = boost::log::expressions;
namespace sinks     = boost::log::sinks;
namespace attrs     = boost::log::attributes;
namespace keywords  = boost::log::keywords;

#endif

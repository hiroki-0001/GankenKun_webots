// **********************************************************************
//
// Copyright (c) 2003-2017 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.7.0
//
// <auto-generated>
//
// Generated from file `StringSender.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#include <StringSender.h>
#include <IceUtil/PushDisableWarnings.h>
#include <Ice/LocalException.h>
#include <Ice/ValueFactory.h>
#include <Ice/OutgoingAsync.h>
#include <Ice/InputStream.h>
#include <Ice/OutputStream.h>
#include <IceUtil/PopDisableWarnings.h>

#if defined(_MSC_VER)
#   pragma warning(disable:4458) // declaration of ... hides class member
#elif defined(__clang__)
#   pragma clang diagnostic ignored "-Wshadow"
#elif defined(__GNUC__)
#   pragma GCC diagnostic ignored "-Wshadow"
#endif

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace
{

const ::std::string iceC_hc_StringSender_ids[2] =
{
    "::Ice::Object",
    "::hc::StringSender"
};
const ::std::string iceC_hc_StringSender_ops[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "sendString"
};
const ::std::string iceC_hc_StringSender_sendString_name = "sendString";

}

bool
hc::StringSender::ice_isA(::std::string s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_hc_StringSender_ids, iceC_hc_StringSender_ids + 2, s);
}

::std::vector<::std::string>
hc::StringSender::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector<::std::string>(&iceC_hc_StringSender_ids[0], &iceC_hc_StringSender_ids[2]);
}

::std::string
hc::StringSender::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
hc::StringSender::ice_staticId()
{
    static const ::std::string typeId = "::hc::StringSender";
    return typeId;
}

bool
hc::StringSender::_iceD_sendString(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    auto istr = inS.startReadParams();
    ::std::string iceP_s;
    istr->readAll(iceP_s);
    inS.endReadParams();
    ::std::string ret = this->sendString(::std::move(iceP_s), current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(ret);
    inS.endWriteParams();
    return true;
}

bool
hc::StringSender::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_hc_StringSender_ops, iceC_hc_StringSender_ops + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_hc_StringSender_ops)
    {
        case 0:
        {
            return _iceD_ice_id(in, current);
        }
        case 1:
        {
            return _iceD_ice_ids(in, current);
        }
        case 2:
        {
            return _iceD_ice_isA(in, current);
        }
        case 3:
        {
            return _iceD_ice_ping(in, current);
        }
        case 4:
        {
            return _iceD_sendString(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}

void
hc::StringSenderPrx::_iceI_sendString(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::std::string>>& outAsync, const ::std::string& iceP_s, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_hc_StringSender_sendString_name);
    outAsync->invoke(iceC_hc_StringSender_sendString_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        [&](::Ice::OutputStream* ostr)
        {
            ostr->writeAll(iceP_s);
        },
        nullptr);
}

::std::shared_ptr<::Ice::ObjectPrx>
hc::StringSenderPrx::_newInstance() const
{
    return ::IceInternal::createProxy<StringSenderPrx>();
}

const ::std::string&
hc::StringSenderPrx::ice_staticId()
{
    return hc::StringSender::ice_staticId();
}

#else // C++98 mapping

namespace
{

const ::std::string iceC_hc_StringSender_sendString_name = "sendString";

}
::IceProxy::Ice::Object* ::IceProxy::hc::upCast(::IceProxy::hc::StringSender* p) { return p; }

void
::IceProxy::hc::_readProxy(::Ice::InputStream* istr, ::IceInternal::ProxyHandle< ::IceProxy::hc::StringSender>& v)
{
    ::Ice::ObjectPrx proxy;
    istr->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::hc::StringSender;
        v->_copyFrom(proxy);
    }
}

::Ice::AsyncResultPtr
IceProxy::hc::StringSender::_iceI_begin_sendString(const ::std::string& iceP_s, const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_hc_StringSender_sendString_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_hc_StringSender_sendString_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_hc_StringSender_sendString_name, ::Ice::Normal, context);
        ::Ice::OutputStream* ostr = result->startWriteParams(::Ice::DefaultFormat);
        ostr->write(iceP_s);
        result->endWriteParams();
        result->invoke(iceC_hc_StringSender_sendString_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

::std::string
IceProxy::hc::StringSender::end_sendString(const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_hc_StringSender_sendString_name);
    ::std::string ret;
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(ret);
    result->_endReadParams();
    return ret;
}

::IceProxy::Ice::Object*
IceProxy::hc::StringSender::_newInstance() const
{
    return new StringSender;
}

const ::std::string&
IceProxy::hc::StringSender::ice_staticId()
{
    return ::hc::StringSender::ice_staticId();
}

hc::StringSender::~StringSender()
{
}

::Ice::Object* hc::upCast(::hc::StringSender* p) { return p; }


namespace
{
const ::std::string iceC_hc_StringSender_ids[2] =
{
    "::Ice::Object",
    "::hc::StringSender"
};

}

bool
hc::StringSender::ice_isA(const ::std::string& s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_hc_StringSender_ids, iceC_hc_StringSender_ids + 2, s);
}

::std::vector< ::std::string>
hc::StringSender::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&iceC_hc_StringSender_ids[0], &iceC_hc_StringSender_ids[2]);
}

const ::std::string&
hc::StringSender::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
hc::StringSender::ice_staticId()
{
#ifdef ICE_HAS_THREAD_SAFE_LOCAL_STATIC
    static const ::std::string typeId = "::hc::StringSender";
    return typeId;
#else
    return iceC_hc_StringSender_ids[1];
#endif
}

bool
hc::StringSender::_iceD_sendString(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    ::Ice::InputStream* istr = inS.startReadParams();
    ::std::string iceP_s;
    istr->read(iceP_s);
    inS.endReadParams();
    ::std::string ret = this->sendString(iceP_s, current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(ret);
    inS.endWriteParams();
    return true;
}

namespace
{
const ::std::string iceC_hc_StringSender_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "sendString"
};

}

bool
hc::StringSender::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_hc_StringSender_all, iceC_hc_StringSender_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_hc_StringSender_all)
    {
        case 0:
        {
            return _iceD_ice_id(in, current);
        }
        case 1:
        {
            return _iceD_ice_ids(in, current);
        }
        case 2:
        {
            return _iceD_ice_isA(in, current);
        }
        case 3:
        {
            return _iceD_ice_ping(in, current);
        }
        case 4:
        {
            return _iceD_sendString(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}

void
hc::StringSender::_iceWriteImpl(::Ice::OutputStream* ostr) const
{
    ostr->startSlice(ice_staticId(), -1, true);
    Ice::StreamWriter< ::hc::StringSender, ::Ice::OutputStream>::write(ostr, *this);
    ostr->endSlice();
}

void
hc::StringSender::_iceReadImpl(::Ice::InputStream* istr)
{
    istr->startSlice();
    Ice::StreamReader< ::hc::StringSender, ::Ice::InputStream>::read(istr, *this);
    istr->endSlice();
}

void
hc::_icePatchObjectPtr(StringSenderPtr& handle, const ::Ice::ObjectPtr& v)
{
    handle = ::hc::StringSenderPtr::dynamicCast(v);
    if(v && !handle)
    {
        IceInternal::Ex::throwUOE(::hc::StringSender::ice_staticId(), v);
    }
}

namespace Ice
{
}

#endif

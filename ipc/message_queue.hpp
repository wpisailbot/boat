// This is a modified version of the Boost IPC message queue.




//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright Ion Gaztanaga 2005-2012. Distributed under the Boost
// Software License, Version 1.0. (See accompanying file
// LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/interprocess for documentation.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef BOOST_INTERPROCESS_MESSAGE_QUEUE_HPP
#define BOOST_INTERPROCESS_MESSAGE_QUEUE_HPP

#include <boost/interprocess/detail/config_begin.hpp>
#include <boost/interprocess/detail/workaround.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/detail/managed_open_or_create_impl.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/detail/utilities.hpp>
#include <boost/interprocess/offset_ptr.hpp>
#include <boost/interprocess/creation_tags.hpp>
#include <boost/interprocess/exceptions.hpp>
#include <boost/interprocess/permissions.hpp>
#include <boost/detail/no_exceptions_support.hpp>
#include <boost/interprocess/detail/type_traits.hpp>
#include <boost/intrusive/pointer_traits.hpp>
#include <boost/type_traits/make_unsigned.hpp>
#include <boost/type_traits/alignment_of.hpp>
#include <boost/intrusive/pointer_traits.hpp>
#include <boost/assert.hpp>
#include <algorithm> //std::lower_bound
#include <cstddef>   //std::size_t
#include <cstring>   //memcpy


//!\file
//!Describes an inter-process message queue. This class allows sending
//!messages between processes and allows blocking, non-blocking and timed
//!sending and receiving.

namespace boost{  namespace interprocess{

namespace ipcdetail
{
   template<class VoidPointer>
   class msg_queue_initialization_func_t;
}

//!A class that allows sending messages
//!between processes.
template<class VoidPointer>
class message_queue_t
{
   /// @cond
   //Blocking modes
   enum block_t   {  blocking,   timed,   non_blocking   };

   message_queue_t();
   /// @endcond

   public:
   typedef VoidPointer                                                 void_pointer;
   typedef typename boost::intrusive::
      pointer_traits<void_pointer>::template
         rebind_pointer<char>::type                                    char_ptr;
   typedef typename boost::intrusive::pointer_traits<char_ptr>::difference_type difference_type;
   typedef typename boost::make_unsigned<difference_type>::type        size_type;

   //!Creates a process shared message queue with name "name". For this message queue,
   //!the maximum number of messages will be "max_num_msg" and the maximum message size
   //!will be "max_msg_size". Throws on error and if the queue was previously created.
   message_queue_t(create_only_t create_only,
                 const char *name,
                 size_type max_num_msg,
                 size_type max_msg_size,
                 const permissions &perm = permissions());

   //!Opens or creates a process shared message queue with name "name".
   //!If the queue is created, the maximum number of messages will be "max_num_msg"
   //!and the maximum message size will be "max_msg_size". If queue was previously
   //!created the queue will be opened and "max_num_msg" and "max_msg_size" parameters
   //!are ignored. Throws on error.
   message_queue_t(open_or_create_t open_or_create,
                 const char *name,
                 size_type max_num_msg,
                 size_type max_msg_size,
                 const permissions &perm = permissions());

   //!Opens a previously created process shared message queue with name "name".
   //!If the queue was not previously created or there are no free resources,
   //!throws an error.
   message_queue_t(open_only_t open_only,
                 const char *name);

   //!Destroys *this and indicates that the calling process is finished using
   //!the resource. All opened message queues are still
   //!valid after destruction. The destructor function will deallocate
   //!any system resources allocated by the system for use by this process for
   //!this resource. The resource can still be opened again calling
   //!the open constructor overload. To erase the message queue from the system
   //!use remove().
   ~message_queue_t();

   //!Sends a message stored in buffer "buffer" with size "buffer_size" in the
   //!message queue with priority "priority". If the message queue is full
   //!the sender is blocked. Throws interprocess_error on error.
   void send (const void *buffer,     size_type buffer_size);

   //!Receives a message from the message queue. The message is stored in buffer
   //!"buffer", which has size "buffer_size". The received message has size
   //!"recvd_size" and priority "priority". If the message queue is empty
   //!the receiver is blocked. Throws interprocess_error on error.
   void receive (void *buffer,           size_type buffer_size,
                 size_type &recvd_size);

   //!Receives a message from the message queue. The message is stored in buffer
   //!"buffer", which has size "buffer_size". The received message has size
   //!"recvd_size" and priority "priority". If the message queue is empty
   //!the receiver is not blocked and returns false, otherwise returns true.
   //!Throws interprocess_error on error.
   bool try_receive (void *buffer,           size_type buffer_size,
                     size_type &recvd_size);

   //!Receives a message from the message queue. The message is stored in buffer
   //!"buffer", which has size "buffer_size". The received message has size
   //!"recvd_size" and priority "priority". If the message queue is empty
   //!the receiver retries until time "abs_time" is reached. Returns true if
   //!the message has been successfully sent. Returns false if timeout is reached.
   //!Throws interprocess_error on error.
   bool timed_receive (void *buffer,           size_type buffer_size,
                       size_type &recvd_size,
                       const boost::posix_time::ptime &abs_time);

   //!Returns the maximum number of messages allowed by the queue. The message
   //!queue must be opened or created previously. Otherwise, returns 0.
   //!Never throws
   size_type get_max_msg() const;

   //!Returns the maximum size of message allowed by the queue. The message
   //!queue must be opened or created previously. Otherwise, returns 0.
   //!Never throws
   size_type get_max_msg_size() const;

   //!Returns the number of messages currently stored.
   //!Never throws
   size_type get_num_msg() const;

   //!Removes the message queue from the system.
   //!Returns false on error. Never throws
   static bool remove(const char *name);

   /// @cond
   private:
   typedef boost::posix_time::ptime ptime;

   friend class ipcdetail::msg_queue_initialization_func_t<VoidPointer>;

   bool do_receive(block_t block,
                   void *buffer,         size_type buffer_size,
                   size_type &recvd_size,
                   const ptime &abs_time);

   bool do_send(
                const void *buffer,      size_type buffer_size
                );

   //!Returns the needed memory size for the shared message queue.
   //!Never throws
   static size_type get_mem_size(size_type max_msg_size, size_type max_num_msg);
   typedef ipcdetail::managed_open_or_create_impl<shared_memory_object, 0, true, false> open_create_impl_t;
   open_create_impl_t m_shmem;
   /// @endcond

   size_type m_cur_msg = 0;
};

/// @cond

namespace ipcdetail {

//!This header is the prefix of each message in the queue
template<class VoidPointer>
class msg_hdr_t
{
   typedef VoidPointer                                                           void_pointer;
   typedef typename boost::intrusive::
      pointer_traits<void_pointer>::template
         rebind_pointer<char>::type                                              char_ptr;
   typedef typename boost::intrusive::pointer_traits<char_ptr>::difference_type  difference_type;
   typedef typename boost::make_unsigned<difference_type>::type                  size_type;

   public:
   size_type               len;     // Message length
   //!Returns the data buffer associated with this this message
   void * data(){ return this+1; }  //
};


//!This header is placed in the beginning of the shared memory and contains
//!the data to control the queue. This class initializes the shared memory
//!in the following way: in ascending memory address with proper alignment
//!fillings:
//!
//!-> mq_hdr_t:
//!   Main control block that controls the rest of the elements
//!
//!-> offset_ptr<msg_hdr_t> index [max_num_msg]
//!   An array of pointers with size "max_num_msg" called index. Each pointer
//!   points to a preallocated message. Elements of this array are
//!   reordered in runtime in the following way:
//!
//!   When the current number of messages is "cur_num_msg", the array
//!   is treated like a circular buffer. Starting from position "cur_first_msg"
//!   "cur_num_msg" in a circular way, pointers point to inserted messages and the rest
//!   point to free messages. Those "cur_num_msg" pointers are
//!   ordered by the priority of the pointed message and by insertion order
//!   if two messages have the same priority. So the next message to be
//!   used in a "receive" is pointed by index [(cur_first_msg + cur_num_msg-1)%max_num_msg]
//!   and the first free message ready to be used in a "send" operation is
//!   [cur_first_msg] if circular buffer is extended from front,
//!   [(cur_first_msg + cur_num_msg)%max_num_msg] otherwise.
//!
//!   This transforms the index in a circular buffer with an embedded free
//!   message queue.
//
//!-> struct message_t
//!   {
//!      msg_hdr_t            header;
//!      char[max_msg_size]   data;
//!   } messages [max_num_msg];
//!
//!   An array of buffers of preallocated messages, each one prefixed with the
//!   msg_hdr_t structure. Each of this message is pointed by one pointer of
//!   the index structure.
template<class VoidPointer>
class mq_hdr_t
{
   typedef VoidPointer                                                     void_pointer;
   typedef msg_hdr_t<void_pointer>                                         msg_header;
   typedef typename boost::intrusive::
      pointer_traits<void_pointer>::template
         rebind_pointer<msg_header>::type                                  msg_hdr_ptr_t;
   typedef typename boost::intrusive::pointer_traits
      <msg_hdr_ptr_t>::difference_type                                     difference_type;
   typedef typename boost::make_unsigned<difference_type>::type            size_type;
   typedef typename boost::intrusive::
      pointer_traits<void_pointer>::template
         rebind_pointer<msg_hdr_ptr_t>::type                              msg_hdr_ptr_ptr_t;
   typedef ipcdetail::managed_open_or_create_impl<shared_memory_object, 0, true, false> open_create_impl_t;

   public:
   //!Constructor. This object must be constructed in the beginning of the
   //!shared memory of the size returned by the function "get_mem_size".
   //!This constructor initializes the needed resources and creates
   //!the internal structures like the priority index. This can throw.
   mq_hdr_t(size_type max_num_msg, size_type max_msg_size)
      : m_max_num_msg(max_num_msg),
         m_max_msg_size(max_msg_size),
         m_cur_num_msg(0)
         ,m_cur_first_msg(0u)
      {  this->initialize_memory();  }

   //!Returns true if the message queue is full
   bool is_full() const
      {  return m_cur_num_msg == m_max_num_msg;  }

   //!Returns true if the message queue is empty
   bool is_empty() const
      {  return !m_cur_num_msg;  }

   //!Frees the top priority message and saves it in the free message list
   void free_top_msg()
      {  --m_cur_num_msg;  }
   typedef msg_hdr_ptr_t *iterator;

   size_type end_pos() const
   {
      const size_type space_until_bufend = m_max_num_msg - m_cur_first_msg;
      return space_until_bufend > m_cur_num_msg
         ? m_cur_first_msg + m_cur_num_msg : m_cur_num_msg - space_until_bufend;
   }

   size_type end_index() const {
     return end_pos() ? end_pos() - 1 : m_max_num_msg - 1;
   }

   //!Returns the inserted message with top priority
   msg_header &top_msg()
   {
      size_type pos = this->end_pos();
      return *mp_index[pos ? --pos : m_max_num_msg - 1];
   }

   //!Returns the inserted message with bottom priority
   msg_header &bottom_msg()
      {  return *mp_index[m_cur_first_msg];   }

   iterator inserted_ptr_begin() const
   {  return &mp_index[m_cur_first_msg]; }

   iterator inserted_ptr_end() const
      {  return &mp_index[this->end_pos()];  }

   msg_header & insert_at_begin()
   {
      //unsigned integer guarantees underflow
      m_cur_first_msg = m_cur_first_msg ? m_cur_first_msg : m_max_num_msg;
      --m_cur_first_msg;
      ++m_cur_num_msg;
      return *mp_index[m_cur_first_msg];
   }

   //!Inserts the first free message in the priority queue
   msg_header & queue_free_msg()
   {
      return this->insert_at_begin();
   }

   //!Returns the number of bytes needed to construct a message queue with
   //!"max_num_size" maximum number of messages and "max_msg_size" maximum
   //!message size. Never throws.
   static size_type get_mem_size
      (size_type max_msg_size, size_type max_num_msg)
   {
      const size_type
		 msg_hdr_align  = ::boost::alignment_of<msg_header>::value,
		 index_align    = ::boost::alignment_of<msg_hdr_ptr_t>::value,
         r_hdr_size     = ipcdetail::ct_rounded_size<sizeof(mq_hdr_t), index_align>::value,
         r_index_size   = ipcdetail::get_rounded_size(max_num_msg*sizeof(msg_hdr_ptr_t), msg_hdr_align),
         r_max_msg_size = ipcdetail::get_rounded_size(max_msg_size, msg_hdr_align) + sizeof(msg_header);
      return r_hdr_size + r_index_size + (max_num_msg*r_max_msg_size) +
         open_create_impl_t::ManagedOpenOrCreateUserOffset;
   }

   //!Initializes the memory structures to preallocate messages and constructs the
   //!message index. Never throws.
   void initialize_memory()
   {
      const size_type
      msg_hdr_align  = ::boost::alignment_of<msg_header>::value,
      index_align    = ::boost::alignment_of<msg_hdr_ptr_t>::value,
         r_hdr_size     = ipcdetail::ct_rounded_size<sizeof(mq_hdr_t), index_align>::value,
         r_index_size   = ipcdetail::get_rounded_size(m_max_num_msg*sizeof(msg_hdr_ptr_t), msg_hdr_align),
         r_max_msg_size = ipcdetail::get_rounded_size(m_max_msg_size, msg_hdr_align) + sizeof(msg_header);

      //Pointer to the index
      msg_hdr_ptr_t *index =  reinterpret_cast<msg_hdr_ptr_t*>
                                 (reinterpret_cast<char*>(this)+r_hdr_size);

      //Pointer to the first message header
      msg_header *msg_hdr   =  reinterpret_cast<msg_header*>
                                 (reinterpret_cast<char*>(this)+r_hdr_size+r_index_size);

      //Initialize the pointer to the index
      mp_index             = index;

      //Initialize the index so each slot points to a preallocated message
      for(size_type i = 0; i < m_max_num_msg; ++i){
         index[i] = msg_hdr;
         msg_hdr  = reinterpret_cast<msg_header*>
                        (reinterpret_cast<char*>(msg_hdr)+r_max_msg_size);
      }
   }

   public:
   //Pointer to the index
   msg_hdr_ptr_ptr_t          mp_index;
   //Maximum number of messages of the queue
   const size_type            m_max_num_msg;
   //Maximum size of messages of the queue
   const size_type            m_max_msg_size;
   //Current number of messages
   size_type                  m_cur_num_msg;
   //Mutex to protect data structures
   interprocess_mutex         m_mutex;
   //Condition block receivers when there are no messages
   interprocess_condition     m_cond_recv;
   //Current start offset in the circular index
   size_type                  m_cur_first_msg;
};


//!This is the atomic functor to be executed when creating or opening
//!shared memory. Never throws
template<class VoidPointer>
class msg_queue_initialization_func_t
{
   public:
   typedef typename boost::intrusive::
      pointer_traits<VoidPointer>::template
         rebind_pointer<char>::type                                    char_ptr;
   typedef typename boost::intrusive::pointer_traits<char_ptr>::difference_type difference_type;
   typedef typename boost::make_unsigned<difference_type>::type        size_type;

   msg_queue_initialization_func_t(size_type maxmsg = 0,
                         size_type maxmsgsize = 0)
      : m_maxmsg (maxmsg), m_maxmsgsize(maxmsgsize) {}

   bool operator()(void *address, size_type, bool created)
   {
      char      *mptr;

      if(created){
         mptr     = reinterpret_cast<char*>(address);
         //Construct the message queue header at the beginning
         BOOST_TRY{
            new (mptr) mq_hdr_t<VoidPointer>(m_maxmsg, m_maxmsgsize);
         }
         BOOST_CATCH(...){
            return false;
         }
         BOOST_CATCH_END
      }
      return true;
   }

   std::size_t get_min_size() const
   {
      return mq_hdr_t<VoidPointer>::get_mem_size(m_maxmsgsize, m_maxmsg)
      - message_queue_t<VoidPointer>::open_create_impl_t::ManagedOpenOrCreateUserOffset;
   }

   const size_type m_maxmsg;
   const size_type m_maxmsgsize;
};

}  //namespace ipcdetail {

template<class VoidPointer>
inline message_queue_t<VoidPointer>::~message_queue_t()
{}

template<class VoidPointer>
inline typename message_queue_t<VoidPointer>::size_type message_queue_t<VoidPointer>::get_mem_size
   (size_type max_msg_size, size_type max_num_msg)
{  return ipcdetail::mq_hdr_t<VoidPointer>::get_mem_size(max_msg_size, max_num_msg);   }

template<class VoidPointer>
inline message_queue_t<VoidPointer>::message_queue_t(create_only_t,
                                    const char *name,
                                    size_type max_num_msg,
                                    size_type max_msg_size,
                                    const permissions &perm)
      //Create shared memory and execute functor atomically
   :  m_shmem(create_only,
              name,
              get_mem_size(max_msg_size, max_num_msg),
              read_write,
              static_cast<void*>(0),
              //Prepare initialization functor
              ipcdetail::msg_queue_initialization_func_t<VoidPointer> (max_num_msg, max_msg_size),
              perm)
{}

template<class VoidPointer>
inline message_queue_t<VoidPointer>::message_queue_t(open_or_create_t,
                                    const char *name,
                                    size_type max_num_msg,
                                    size_type max_msg_size,
                                    const permissions &perm)
      //Create shared memory and execute functor atomically
   :  m_shmem(open_or_create,
              name,
              get_mem_size(max_msg_size, max_num_msg),
              read_write,
              static_cast<void*>(0),
              //Prepare initialization functor
              ipcdetail::msg_queue_initialization_func_t<VoidPointer> (max_num_msg, max_msg_size),
              perm)
{}

template<class VoidPointer>
inline message_queue_t<VoidPointer>::message_queue_t(open_only_t, const char *name)
   //Create shared memory and execute functor atomically
   :  m_shmem(open_only,
              name,
              read_write,
              static_cast<void*>(0),
              //Prepare initialization functor
              ipcdetail::msg_queue_initialization_func_t<VoidPointer> ())
{}

template<class VoidPointer>
inline void message_queue_t<VoidPointer>::send
   (const void *buffer, size_type buffer_size)
{  this->do_send(buffer, buffer_size); }

template<class VoidPointer>
inline bool message_queue_t<VoidPointer>::do_send(
                                const void *buffer,      size_type buffer_size)
{
   ipcdetail::mq_hdr_t<VoidPointer> *p_hdr = static_cast<ipcdetail::mq_hdr_t<VoidPointer>*>(m_shmem.get_user_address());
   //Check if buffer is smaller than maximum allowed
   if (buffer_size > p_hdr->m_max_msg_size) {
      throw interprocess_exception(size_error);
   }

   //---------------------------------------------
   scoped_lock<interprocess_mutex> lock(p_hdr->m_mutex);
   //---------------------------------------------
   {
      //If the queue is full execute blocking logic
      if (p_hdr->is_full()) {
        p_hdr->free_top_msg();
      }

      //Insert the first free message in the priority queue
      ipcdetail::msg_hdr_t<VoidPointer> &free_msg_hdr = p_hdr->queue_free_msg();

      //Copy control data to the free message
      free_msg_hdr.len      = buffer_size;

      //Copy user buffer to the message
      std::memcpy(free_msg_hdr.data(), buffer, buffer_size);
   }  // Lock end

   //Notify outside lock to avoid contention. This might produce some
   //spurious wakeups, but it's usually far better than notifying inside.
   //If this message changes the queue empty state, notify it to receivers
   p_hdr->m_cond_recv.notify_all();

   return true;
}

template<class VoidPointer>
inline void message_queue_t<VoidPointer>::receive(void *buffer,        size_type buffer_size,
                        size_type &recvd_size)
{  this->do_receive(blocking, buffer, buffer_size, recvd_size, ptime()); }

template<class VoidPointer>
inline bool
   message_queue_t<VoidPointer>::try_receive(void *buffer,              size_type buffer_size,
                              size_type &recvd_size)
{  return this->do_receive(non_blocking, buffer, buffer_size, recvd_size, ptime()); }

template<class VoidPointer>
inline bool
   message_queue_t<VoidPointer>::timed_receive(void *buffer,            size_type buffer_size,
                                size_type &recvd_size,
                                const boost::posix_time::ptime &abs_time)
{
   if(abs_time == boost::posix_time::pos_infin){
      this->receive(buffer, buffer_size, recvd_size);
      return true;
   }
   return this->do_receive(timed, buffer, buffer_size, recvd_size, abs_time);
}

template<class VoidPointer>
inline bool
   message_queue_t<VoidPointer>::do_receive(block_t block,
                          void *buffer,            size_type buffer_size,
                          size_type &recvd_size,
                          const boost::posix_time::ptime &abs_time)
{
   ipcdetail::mq_hdr_t<VoidPointer> *p_hdr = static_cast<ipcdetail::mq_hdr_t<VoidPointer>*>(m_shmem.get_user_address());
   //Check if buffer is big enough for any message
   if (buffer_size < p_hdr->m_max_msg_size) {
      throw interprocess_exception(size_error);
   }

   //---------------------------------------------
   scoped_lock<interprocess_mutex> lock(p_hdr->m_mutex);
   //---------------------------------------------
   {
      //If there are no messages execute blocking logic
      if (p_hdr->is_empty() || m_cur_msg == p_hdr->m_cur_first_msg) {
         switch(block){
            case non_blocking :
               return false;
            break;

            case blocking :
               do{
                  p_hdr->m_cond_recv.wait(lock);
                  m_cur_msg = p_hdr->m_cur_first_msg + 1;
               }
               while (p_hdr->is_empty());
            break;

            case timed :
               do{
                  if(!p_hdr->m_cond_recv.timed_wait(lock, abs_time)){
                     if(p_hdr->is_empty())
                        return false;
                     break;
                  }
               }
               while (p_hdr->is_empty());
            break;

            //Paranoia check
            default:
            break;
         }
      }

      m_cur_msg = m_cur_msg ? m_cur_msg - 1 : p_hdr->m_max_num_msg - 1;

      if (p_hdr->end_index() >= p_hdr->m_cur_first_msg) {
        if (m_cur_msg > p_hdr->end_index() ||
            m_cur_msg < p_hdr->m_cur_first_msg) {
          m_cur_msg = p_hdr->end_index();
        }
      } else {
        if (m_cur_msg > p_hdr->end_index() &&
            m_cur_msg < p_hdr->m_cur_first_msg) {
          m_cur_msg = p_hdr->end_index();
        }
      }

      //There is at least one message ready to pick, get the top one
      ipcdetail::msg_hdr_t<VoidPointer> &top_msg =
          *p_hdr->mp_index[m_cur_msg];

      //Get data from the message
      recvd_size     = top_msg.len;

      //Copy data to receiver's bufers
      std::memcpy(buffer, top_msg.data(), recvd_size);
   }  //Lock end

   return true;
}

template<class VoidPointer>
inline typename message_queue_t<VoidPointer>::size_type message_queue_t<VoidPointer>::get_max_msg() const
{
   ipcdetail::mq_hdr_t<VoidPointer> *p_hdr = static_cast<ipcdetail::mq_hdr_t<VoidPointer>*>(m_shmem.get_user_address());
   return p_hdr ? p_hdr->m_max_num_msg : 0;  }

template<class VoidPointer>
inline typename message_queue_t<VoidPointer>::size_type message_queue_t<VoidPointer>::get_max_msg_size() const
{
   ipcdetail::mq_hdr_t<VoidPointer> *p_hdr = static_cast<ipcdetail::mq_hdr_t<VoidPointer>*>(m_shmem.get_user_address());
   return p_hdr ? p_hdr->m_max_msg_size : 0;
}

template<class VoidPointer>
inline typename message_queue_t<VoidPointer>::size_type message_queue_t<VoidPointer>::get_num_msg() const
{
   ipcdetail::mq_hdr_t<VoidPointer> *p_hdr = static_cast<ipcdetail::mq_hdr_t<VoidPointer>*>(m_shmem.get_user_address());
   if(p_hdr){
      //---------------------------------------------
      scoped_lock<interprocess_mutex> lock(p_hdr->m_mutex);
      //---------------------------------------------
      return p_hdr->m_cur_num_msg;
   }

   return 0;
}

template<class VoidPointer>
inline bool message_queue_t<VoidPointer>::remove(const char *name)
{  return shared_memory_object::remove(name);  }

/// @endcond

}} //namespace boost{  namespace interprocess{

#include <boost/interprocess/detail/config_end.hpp>

#endif   //#ifndef BOOST_INTERPROCESS_MESSAGE_QUEUE_HPP

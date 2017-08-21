#ifndef BUFFER_H
#define BUFFER_H

#include <iostream>
#include <semaphore.h>
#include <pthread.h>
#include <queue>
#include <mutex>
#include <condition_variable>

using namespace std;

template<class T> class Buffer
{
public:
    explicit Buffer(int size);
    void add(const T& data, bool dropIfFull = false);
    T get();
    bool clear();
    int size() const
    {
        return m_queue.size();
    }
    int maxSize() const
    {
        return m_bufferSize;
    }
    bool isFull() const
    {
        return m_queue.size() == m_bufferSize;
    }
    bool isEmpty() const
    {
        return m_queue.size() == 0;
    }
    bool isNotFull()
    {
        return m_queue.size() < m_bufferSize;
    }
    bool isNotEmpty()
    {
        return m_queue.size() > 0;
    }

private:
    queue<T> m_queue;
    mutex m_queueProtectMutex;
    mutex m_mutex;
    mutex m_addProtectMutex;
    mutex m_getProtectMutex;
    condition_variable m_bufferNotFullCV;
    condition_variable m_bufferNotEmptyCV;
    int m_bufferSize;
};

template<class T> Buffer<T>::Buffer(int size)
{
    //Save buffer size
    m_bufferSize = size;
}

template<class T> void Buffer<T>::add(const T& data, bool dropIfFull)
{
    //m_bufferNotFullCV.wait(lck, [&]{ return isNotFull(); });
    m_addProtectMutex.lock();

    //If dropping is enabled, don't block if buffer is full
    if(dropIfFull)
    {
        //Buffer is not full
        unique_lock<mutex> lck(m_mutex);
        if (isNotFull())
        {
            lck.unlock();

            //Add item to queue
            m_queueProtectMutex.lock();
            m_queue.push(data);
            m_queueProtectMutex.unlock();

            //Signal that buffer is not empty
            m_mutex.lock();
            m_bufferNotEmptyCV.notify_one();
            m_mutex.unlock();
        }
        //Buffer is full, do nothing
        else {
            lck.unlock();
        }
    }
    //If buffer is full, wait for bufferNotFull
    else
    {
        unique_lock<mutex> lck(m_mutex);
        //If buffer is full, wait on isNotFull()
        if (isFull()) {
            m_bufferNotFullCV.wait(lck);
        }
        lck.unlock();

        //Add item to queue
        m_queueProtectMutex.lock();
        m_queue.push(data);
        m_queueProtectMutex.unlock();

        //Signal that buffer is not empty
        m_mutex.lock();
        m_bufferNotEmptyCV.notify_one();
        m_mutex.unlock();
    }

    m_addProtectMutex.unlock();
}

template<class T> T Buffer<T>::get()
{
    T data;
    m_getProtectMutex.lock();

    unique_lock<mutex> lck(m_mutex);
    //If buffer is empty, wait on isNotEmpty()
    if (isEmpty()) {
        m_bufferNotEmptyCV.wait(lck);
    }
    lck.unlock();

    //Take item from queue
    m_queueProtectMutex.lock();
    data = m_queue.front();
    m_queue.pop();
    m_queueProtectMutex.unlock();

    //Signal that buffer is not full
    m_mutex.lock();
    m_bufferNotFullCV.notify_one();
    m_mutex.unlock();

    m_getProtectMutex.unlock();
    return data;
}

template<class T> bool Buffer<T>::clear()
{
    //Check if buffer contains items
    if (m_queue.size() > 0)
    {
        //Stop adding items to buffer (will return false if an item is currently being added to the buffer)
        if (m_addProtectMutex.try_lock())
        {
            //Stop taking items from buffer (will return false if an item is currently being taken from the buffer)
            if (m_getProtectMutex.try_lock())
            {
                // Clear buffer
                m_queue = queue<T>();

                // Allow get method to resume
                m_getProtectMutex.unlock();
            }
            else
            {
                //Allow add method to resume
                m_addProtectMutex.unlock();
                return false;
            }
            //Allow add method to resume
            m_addProtectMutex.unlock();
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}



#endif // BUFFER_H
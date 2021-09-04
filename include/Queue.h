

#include <queue>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
 
template <class T, class Container = std::vector<T>, class Compare = std::less<typename Container::value_type> >
class Queue
{
 public:

  T pop()
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    auto item = queue_.top();
    queue_.pop();
    return item;
  }
 
  void pop(T& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    item = queue_.top();
    queue_.pop();
  }
 
  void push(const T& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(item);
    mlock.unlock();
    cond_.notify_one();
  }
 
  void push(T&& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(std::move(item));
    mlock.unlock();
    cond_.notify_one();
  }

  bool empty() 
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    return queue_.empty();
  }

    int size() 
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    return queue_.size();
  }

 private:
  std::priority_queue<T,Container,Compare> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
};
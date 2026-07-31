#pragma once

#include "Core.h"

#include "NonCopyable.h"

#define LOCKFREE_CAS_QUEUE 1

#if LOCKFREE_CAS_QUEUE
#include <array> ///using Task buffer array 
#else
#include <queue> /// using std::deque
#endif // LOCKFREE_CAS_QUEUE

#include <thread>
#include <functional>
#include <mutex>


#define DEBUG_THREAD 0
#define ENABLE_THREAD_LOG 0

#define USE_TASK_CONSTURCT_BUFF 1

#define PRESISTENT_TASK_SIGNAL 0


#if ENABLE_THREAD_LOG
#define THREAD_LOG_MSG(msg) \
		TaskCoordinatorMt::mThreadLogMutex.lock(); \
		LOG_MSG(msg); \
		TaskCoordinatorMt::mThreadLogMutex.unlock(); 
#else
#define THREAD_LOG_MSG(msg)
#endif // ENABLE_THREAD_LOG



namespace vx {

	class TaskCoordinator;
	using TaskFunction = std::function<void()>;




	class Task
	{
	public:
		Task() = delete;
		Task(TaskCoordinator* coordinator, const TaskFunction& _func, uint32_t _dependancies) :
			mFunc(_func), mCoordinator(coordinator), mDependencies(_dependancies) {
		}

		void IncrementDependency() { mDependencies.fetch_add(1, std::memory_order_relaxed); }
		void RemoveDependency();

		bool Process()
		{
			if (mDependencies.load(std::memory_order_relaxed) > 0)
				return false;
			mFunc();
			//successfully complete job
			return true;
		}
	private:
		friend TaskCoordinator;
		bool DecrementDependency();

		TaskFunction mFunc;
		TaskCoordinator* mCoordinator = nullptr;
		std::atomic<uint32_t> mDependencies = 0;
	};


	class TaskHandle
	{
	public:

		void AddDependee(Task* task)
		{
			if (mComplete)
				task->RemoveDependency();

			mDependees.push_back(task);
		}

		void NotifyDependees()
		{
			for (auto& _task : mDependees)
				_task->RemoveDependency();
		}

		void OnComplete()
		{
			NotifyDependees();
			mComplete = true;
		}
	private:
		Task* mTask;
		std::vector<Task*> mDependees;
		bool mComplete = false;
	};


	class TaskCoordinator : public NonCopyable
	{
	public:
		TaskCoordinator() = default;
		virtual ~TaskCoordinator() = default;

		virtual Task* ConstructTask(const TaskFunction& task_func, uint32_t dependencies) = 0;
		virtual void EnqueueTask(Task* task) = 0;
		virtual void EnqueueTasks(Task** tasks, uint32 count) = 0;

		void RemoveTaskDependency(Task* task)
		{
			// check if task is part of coordinator 
			/// const uint8* _addr = reinterpret_cast<const uint8*>(addr);
			/// return _addr >= mMemStart && _addr < mMemStart + mStackSize;
			if (task->DecrementDependency())
				//now we can add task
				EnqueueTask(task);
		}

		void RemoveTasksDependency(Task** tasks, uint32_t task_count);

		//using ParallelRangeFunc = std::function<void(void*,uint32, uint32)>;
		struct RangeTask
		{
			void* userData = nullptr;
			void(*Execute)(void*, uint32 begin, uint32 end) = nullptr;
		};
		virtual void ParallelFor(uint32 count, uint32 batch_size, const RangeTask& task) = 0;

		virtual void Quit() = 0;
		virtual void WaitForTasks() = 0;

	protected:
		__forceinline void ImmediateTaskProcess(TaskFunction func) const { func(); }
		__forceinline void ImmediateTaskProcess(Task* _task) const { _task->mFunc(); }
	};

	class TaskCoordinatorSequential : public TaskCoordinator
	{
	public:
		TaskCoordinatorSequential() = default;
		~TaskCoordinatorSequential() = default;

		Task* ConstructTask(const TaskFunction& task_func, uint32_t dependencies) override
		{
			Task* t = nullptr;
			if (dependencies == 0)
				ImmediateTaskProcess(task_func);
			else ///if task has dependancies, construct task
				t = new Task(this, task_func, dependencies);
			return t;
		}

		void EnqueueTask(Task* task) override { ImmediateTaskProcess(task); delete task; }
		void EnqueueTasks(Task** tasks, uint32_t count);

		void Quit() override {}
		void WaitForTasks() override {}
	};

	class TaskCoordinatorMt : public TaskCoordinator
	{
	public:

		TaskCoordinatorMt(int thread_count = -1) //: mNumThread(thread_count) 
		{
			BeginThreads(thread_count);
#if USE_TASK_CONSTURCT_BUFF
			mTaskBuffer.Init(kMaxTaskQueue * 2);
#endif // USE_TASK_CONSTURCT_BUFF
		}

		~TaskCoordinatorMt() { EndThreads(); }

		Task* ConstructTask(const TaskFunction& task_func, uint32_t dependencies) override;

		void EnqueueTask(Task* task) override;
		void EnqueueTasks(Task** tasks, uint32_t count) override;

		/// behaves as a sub task, where a thread/task 
		/// break works down to be process 
		/// 
		/// no dependacies, support dependacies later  

		void ParallelFor(uint32 count, uint32 batch_size, const RangeTask& task) override;

		void Quit() override
		{
			mOnQuitThreads = true;
			mTaskAvailable.notify_all(); //notify all using this flag on mOnQuitThreads

			EndThreads();
		}


		void WaitForTasks() override;
	private:
		void ThreadsMainLoop(int thread_worker_idx);

		/// if thread count is 0 or less, 
		/// the coordinator use max hardward supported count
		void BeginThreads(int thread_count);
		/// Terminates threads
		void EndThreads();

		Task* ConstructTaskInternal(TaskFunction task_func, uint32_t dependencies)
		{
#if USE_TASK_CONSTURCT_BUFF
			return mTaskBuffer.ConstructPtr(this, task_func, dependencies);
#else
			return new Task(this, task_func, dependencies);
#endif // USE_TASK_CONSTURCT_BUFF
		}

		__forceinline void SignalAvailableTask(size_t task_count)
		{
			if (task_count > 1)
				mTaskAvailable.notify_all();
			else if (task_count == 1)
				mTaskAvailable.notify_one();

			///// other thread makes use of a token so orignal token needs to be greater than 2
			//if (mMainThreadWaitingTask && task_count > 2)
			//	mMainWaitFlag.notify_one();

			///for now always wake main thread
			if (mMainThreadWaitingTask)
				mMainWaitFlag.notify_one();
		}

		__forceinline void SignalMainThread() { mMainWaitFlag.notify_one(); }

		__forceinline void PokeWorkers()
		{
			mTaskAvailable.notify_all();
			mMainWaitFlag.notify_one();
		}

		/// it should be easy to check if a task belong to 
		/// task coordinator. 
		/// by using it pointer address 
		/// hence 
		/// we have the begin of allo and end 
		/// as a linear allocator 
		/// if the address is within the allocation 
		/// region then this task belong to task coordinator 
	private:
		bool mOnQuitThreads = false;
		static constexpr int kMaxThreads = 32;
		std::vector<std::thread> mWorkers;


		bool mQuickHackThreadCatchUp = false;

#if ENABLE_THREAD_LOG
		static std::mutex mThreadLogMutex;
#endif // ENABLE_THREAD_LOG


		std::mutex mTaskQueueMutex;
		/// flag for available/pending tasks
		std::condition_variable mTaskAvailable;
		std::condition_variable mMainWaitFlag;




		/// pending tasks
		static constexpr uint32_t kMaxTaskQueue = 1024; //need to be power of 2 to support wrapping
#if LOCKFREE_CAS_QUEUE

		std::array<std::atomic<Task*>, kMaxTaskQueue> mPendingTasks;
		std::atomic<uint32_t> mPendingTasksTail = 0;
		std::atomic<uint32_t> mAvailableTaskCount = 0;

		/// mainly used to access the Main thread task head
		/// num of worker threads excluding main thread 
		uint32_t mNumWorkerThread = 0;
		/// num of active worker thread 
		/// == mNumWorkerThread + MainThread, when main thread is waiting and processing task
		uint32_t mNumActiveWorkerThread = 0;

		std::atomic<uint32_t> mThreadTaskHead[kMaxThreads + 1]; //extra space to main thread, during task waiting
		static_assert(std::is_integral_v<std::remove_reference_t<decltype(mThreadTaskHead[0])>::value_type>);
		static_assert(sizeof(mThreadTaskHead[0]) == 4);

		/// min head, based on the firtst/small thread head
		/// to ensure task gravitates around thread scan window
		/// reduncing threads from wasting clock cyles scanning empty 
		/// mem region
		uint32_t ComputeMinThreadHead()
		{
			///just to check if main thread is participating
			//uint32_t workers = (mMainThreadWaitingTask) ? mWorkers.size() + 1 : mWorkers.size();
			uint32_t workers = mNumActiveWorkerThread;
			uint32_t temp = mThreadTaskHead[0];
			for (int i = 1; i < workers; ++i) //+1 as the main thread uses the slot to last work thread
				temp = std::min(temp, mThreadTaskHead[i].load());
			return temp;
		}
#else
		std::deque<Task*> mTasks;
#endif // LOCKFREE_CAS_QUEUE



		/// might need a mutex for task construct 
		/// or move the mTaskQueueMutex to be task construct 
		/// mutex, so one thread can access at a time, 
		/// 
		/// but then the queue, to use atomics then 
		/// later after learning more about 
		/// compare and exchange to make lock free
		class TaskBuffer : public NonCopyable
		{
		private:
			struct BufferData
			{
				Task taskData;
				/// so instead of have a list of free when have a chain of free
				/// buffer points to the first/head of chain
				/// so we would have to manage multiple datas
				std::atomic<uint32_t> nextFree;
			};
			static_assert(alignof(BufferData) == alignof(Task));
			static_assert(offsetof(BufferData, taskData) == 0, "");
		public:
			void Init(size_t max_task)
			{
				mMax = static_cast<uint32_t>(max_task);
				//mBuffer = new BufferData[max_task]; //no default constructot 
				mBuffer = reinterpret_cast<BufferData*>(new uint8_t[sizeof(BufferData) * max_task]);
			}

			~TaskBuffer()
			{
				//delete[] mBuffer;
				delete[] reinterpret_cast<uint8_t*>(mBuffer);
			}

			/// just in case
			template<typename ...Params>
			uint32_t Construct(Params &&... params)
			{
				for (;;)
				{
					///this remove the index (the slot) 
					/// of the constructed data in list so 
					/// its easy to get the data
					/// 
					/// lets try the free list
					/// capute the next free slot
					uint32_t capture_nxt_free = mFirstNextFree.load(std::memory_order_acquire);
					/// either the failed to free list / on free list 
					if (capture_nxt_free == kInvalidSlot)
					{
						///need to move pointer up
						uint32_t top = mBufferTop.load(std::memory_order_relaxed);
						if (top >= mMax)return kInvalidSlot;

						if (mBufferTop.compare_exchange_weak(top, top + 1, std::memory_order_acquire, std::memory_order_relaxed))
						{
							BufferData& buff_data = mBuffer[top];
							new (&buff_data.taskData) Task(std::forward<Params>(params)...);
							buff_data.nextFree.store(top, std::memory_order_relaxed);
							return top;
						}
					}
					else
					{
						///we found a free slot 
						///has a successful slot 
						BufferData& buff_data = mBuffer[capture_nxt_free];
						uint32_t nxt_nxt_slot = buff_data.nextFree.load(std::memory_order_acquire);
						///try CAS, as to prevent race condition
						if (mFirstNextFree.compare_exchange_weak(capture_nxt_free, nxt_nxt_slot, std::memory_order_acquire, std::memory_order_relaxed)) //ensure fail, expect old cap, desire new (nxt nxt)
						{
							new (&buff_data.taskData) Task(std::forward<Params>(params)...);
							///slot ?(next free), point to self for easy deconstruct via Task* 
							buff_data.nextFree.store(capture_nxt_free, std::memory_order_relaxed);
							return capture_nxt_free;
						}
					}
				}
			}

			template<typename ...Params>
			Task* ConstructPtr(Params&&... params) { return Get(Construct(std::forward<Params>(params)...)); }


			void Deconstruct(uint32_t slot_idx);
			void Deconstruct(Task* task);

			/// need to be able to 
			/// be align to that the address of 
			/// BufferData == Task (its task) 
			/// for easy conversion
			Task* Get(uint32_t idx) const { return &mBuffer[idx].taskData; }
		private:
			BufferData* mBuffer = nullptr;
			uint32_t mMax = 0;
			uint32_t kInvalidSlot = 0xffffffff;
			std::atomic<uint32_t> mFirstNextFree{ 0xffffffff };
			std::atomic<uint32_t> mBufferTop{ 0 };
		};

		TaskBuffer mTaskBuffer;
		bool mMainThreadWaitingTask = false;

		/// current task been processed by threads
		std::atomic<int> mProcessingTasks = 0;
	};



} ///namespace vx
/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <vilo/global.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <iostream>

#define MAPPING_THREADS 4


struct RunningStats
{
	int n_updates;
	int n_failed_matches;
	int n_out_views;
	int n_seeds;

	int n_fail_triangulation;
	int n_fail_score;
	int n_fail_alignment;
	int n_fail_lsd;

	inline RunningStats()
	{
		setZero();
	}

	inline void setZero()
	{
		memset(this,0,sizeof(RunningStats));
	}

	inline void add(RunningStats* r)
	{
		int* pt = (int*)this;
		int* pt_r = (int*)r;
		for(int i=0;i<static_cast<int>(sizeof(RunningStats)/sizeof(int));i++)
			pt[i] += pt_r[i];
	}
};

namespace lsd_slam
{

class IndexThreadReduce
{

public:
	inline IndexThreadReduce()
	{
		nextIndex = 0;
		maxIndex = 0;
		stepSize = 1;
		callPerIndex = boost::bind(&IndexThreadReduce::callPerIndexDefault, this, _1, _2, _3);

		running = true;
		for(int i=0;i<MAPPING_THREADS;i++)
		{
			isDone[i] = false;
			workerThreads[i] = boost::thread(&IndexThreadReduce::workerLoop, this, i);
		}

	}
	inline ~IndexThreadReduce()
	{
		running = false;

		exMutex.lock();
		todo_signal.notify_all();
		exMutex.unlock();

		for(int i=0;i<MAPPING_THREADS;i++)
			workerThreads[i].join();

	}

	inline void reduce(boost::function<void(int,int,RunningStats*)> callPerIndex, int first, int end, RunningStats* stats, int stepSize = 0)
	{
		runningStats = stats;

		if(stepSize == 0)
			stepSize = ((end-first)+MAPPING_THREADS-1)/MAPPING_THREADS;

		boost::unique_lock<boost::mutex> lock(exMutex);

		this->callPerIndex = callPerIndex;
		nextIndex = first;
		maxIndex = end;
		this->stepSize = stepSize;

		for(int i=0;i<MAPPING_THREADS;i++)
			isDone[i] = false;

		todo_signal.notify_all();

		while(true)
		{
			done_signal.wait(lock);
			bool allDone = true;
			for(int i=0;i<MAPPING_THREADS;i++)
				allDone = allDone && isDone[i];

			if(allDone)
				break;
		}

		nextIndex = 0;
		maxIndex = 0;
		this->callPerIndex = boost::bind(&IndexThreadReduce::callPerIndexDefault, this, _1, _2, _3);

	}


private:
	boost::thread workerThreads[MAPPING_THREADS];
	bool isDone[MAPPING_THREADS];

	boost::mutex exMutex;
	boost::condition_variable todo_signal;
	boost::condition_variable done_signal;

	int nextIndex;
	int maxIndex;
	int stepSize;

	RunningStats* runningStats;

	bool running;

	boost::function<void(int,int,RunningStats*)> callPerIndex;

	void callPerIndexDefault(int i, int j, RunningStats* k)
	{
		printf("ERROR: should never be called....\n");
	}

	void workerLoop(int idx)
	{
		boost::unique_lock<boost::mutex> lock(exMutex);

		while(running)
		{
			// try to get something to do.
			int todo = 0;
			bool gotSomething = false;
			if(nextIndex < maxIndex)
			{
				// got something!
				todo = nextIndex;
				nextIndex+=stepSize;
				gotSomething = true;
			}

			// if got something: do it (unlock in the meantime)
			if(gotSomething)
			{
				lock.unlock();

				assert(callPerIndex != 0);

				RunningStats* s = new RunningStats();
				callPerIndex(todo, std::min(todo+stepSize, maxIndex), s);

				lock.lock();
				runningStats->add(s);
			}

			// otherwise wait on signal, releasing lock in the meantime.
			else
			{
				isDone[idx] = true;
				done_signal.notify_all();
				todo_signal.wait(lock);
			}
		}
	}
};
} 

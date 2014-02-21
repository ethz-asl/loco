/*
 * GaitAPS.cpp
 *
 *  Created on: Aug 22, 2013
 *      Author: gech
 */

#include "GaitAPS.hpp"
#include <cstdio>

namespace loco {

GaitAPS::GaitAPS() {

	for (int iLeg=0; iLeg<4; iLeg++)  {
		stancePhases_[iLeg] = 0.0;
		swingPhases_[iLeg] = -1.0;
	}

}

GaitAPS::~GaitAPS() {

}

void GaitAPS::initAPS(APS &aps, double dt)
{
	/* delete all APS */
	listAPS_.clear();

	/* previous APS */
	aps.startTime_ = 0.0;
	listAPS_.push_back(aps);
	prevAPS[0] =  prevAPS[1] = prevAPS[2] = prevAPS[3] = --listAPS_.end();

	/* current APS */
	aps.startTime_ += aps.foreCycleDuration_;
	listAPS_.push_back(aps);
	currentAPS[0] =  currentAPS[1] = currentAPS[2] = currentAPS[3]  = --listAPS_.end();

	/* next APS */
	aps.startTime_ += aps.foreCycleDuration_;
	listAPS_.push_back(aps);
	nextAPS[0] = nextAPS[1] = nextAPS[2] = nextAPS[3] = --listAPS_.end();

	/* second next APS */
	nextNextAPS[0] = nextNextAPS[1] = nextNextAPS[2] = nextNextAPS[3] = --listAPS_.end();

	/* update time */
    time_ = currentAPS[0]->startTime_;

    /* run one cycle to initialize all legs correctly */
	const int nSteps = (int) 1*currentAPS[0]->foreCycleDuration_/dt;
	for (int i=0; i<nSteps ;i++) {
		advance(dt);
	}

	numGaitCycles = 0;
}

double GaitAPS::getStrideDuration()
{
	return currentAPS[0]->foreCycleDuration_;
}

void GaitAPS::setStrideDuration(double strideDuration)
{
	currentAPS[0]->foreCycleDuration_ = strideDuration;
	printf("WARNING: hindCycleDuration is not adapted!\n");
}

void GaitAPS::advance(double dt)
{
	// update time
	time_ += dt;

	/* debug */
//	printf("time_: %lf aps list size: %d\n", time, listAPS_.size());
//	int i=0;
//	for (APSIterator it=listAPS_.begin(); it!=listAPS_.end(); it++, i++) {
//		printf("APS %d: startTime: %lf phase: %lf\n",i, it->startTime_, it->phase_);
//	}

	/* update first leg that guides the timing of the APS */
	if (currentAPS[0]->foreCycleDuration_ != 0.0) {
		currentAPS[0]->phase_ = currentAPS[0]->phase_ + dt/currentAPS[0]->foreCycleDuration_;
	} else {
		currentAPS[0]->phase_ = 1.1;
	}

	if (currentAPS[0]->phase_ > 1) {
		// end of APS
		currentAPS[0]->phase_ = 0.0;
		/* move all pointers forward */
		currentAPS[0]++;
		nextAPS[0]++;
		prevAPS[0]++;
		nextNextAPS[0]++;
		time_ = currentAPS[0]->startTime_; // correct time because it could drift
		/* check if next APS exists in the list */
		if (nextAPS[0] == listAPS_.end()) {
			/* APS does not exist -> copy current APS */
			listAPS_.push_back((*currentAPS[0]));
			nextAPS[0] = nextNextAPS[0] = --listAPS_.end();

			nextAPS[0]->startTime_ = currentAPS[0]->startTime_ + currentAPS[0]->foreCycleDuration_;

		}


		/* remove first APS in the list if it is not needed anymore by any leg */
		bool removeFirstAPSInList = true;
		for (int i=0; i<4; i++) {
			if (listAPS_.begin() == prevAPS[i]) {
				removeFirstAPSInList = false;
				break;
			}
		}
		if ( removeFirstAPSInList) {
			listAPS_.pop_front();
		}
//		printf("Number of APS: %d\n", (int) listAPS_.size());
		// update number of cycles
		numGaitCycles++;

		/* debug */
//		printf("numGaitCycles: %ld\n",numGaitCycles);
//		APSIterator it = currentAPS[0];
//		printf("-----\nCurrent APS:\n");
//		printf("phase: %lf\n", it->phase_);
//		printf("startTime: %lf\n", it->startTime_);
//		printf("cycleDuration: %lf\n",it->cycleDuration_);
//		printf("foreDutyFactor: %lf\n", it->foreDutyFactor_);
//		printf("hindDutyFactor: %lf\n", it->hindDutyFactor_);
//		printf("foreLag: %lf\n", it->foreLag_);
//		printf("hindLag: %lf\n", it->hindLag_);
//		printf("pairLag: %lf\n", it->pairLag_);
//		printf("interpolate: %lf\n", it->interpolate_);
//		print();


	}



	/* update phases of first leg */
	stancePhases_[0] = currentAPS[0]->phase_/currentAPS[0]->foreDutyFactor_;
	if (stancePhases_[0] > 1 || stancePhases_[0] < 0) {
		stancePhases_[0] = 0.0;
	}
	const double swingDuration = 1.0-currentAPS[0]->foreDutyFactor_;
	if (swingDuration != 0 ) {
		swingPhases_[0] = (currentAPS[0]->phase_-currentAPS[0]->foreDutyFactor_)/(swingDuration);
	} else {
		swingPhases_[0] =-1.0;
	}

	if (swingPhases_[0] > 1 || swingPhases_[0] < 0) {
		swingPhases_[0] = -1.0;
	}


	double currentAPSStartTime;
	double currentAPSStanceTimeStart;
	double currentAPSStanceTimeEnd;
	double currentAPSEndTime;

	double nextAPSStartTime;
	double nextAPSStanceTimeStart;
	double nextAPSStanceTimeEnd;
	double nextAPSEndTime;


	double nextNextAPSStartTime;
	double nextNextAPSStanceTimeStart;
	double nextNextAPSStanceTimeEnd;
	double nextNextAPSEndTime;


	double prevAPSStartTime;
	double prevAPSStanceTimeStart;
	double prevAPSStanceTimeEnd;
	double prevAPSEndTime;

	for (int iLeg=1; iLeg<4; iLeg++) {
		currentAPS[iLeg]->getAPSTimesForLeg(iLeg, currentAPSStartTime, currentAPSEndTime, currentAPSStanceTimeStart, currentAPSStanceTimeEnd);
		nextAPS[iLeg]->getAPSTimesForLeg(iLeg, nextAPSStartTime, nextAPSEndTime, nextAPSStanceTimeStart, nextAPSStanceTimeEnd);

		prevAPS[iLeg]->getAPSTimesForLeg(iLeg, prevAPSStartTime, prevAPSEndTime, prevAPSStanceTimeStart, prevAPSStanceTimeEnd);
		nextNextAPS[iLeg]->getAPSTimesForLeg(iLeg, nextNextAPSStartTime, nextNextAPSEndTime, nextNextAPSStanceTimeStart, nextNextAPSStanceTimeEnd);


		/* debug */
//		if (currentAPS[0]->phase_ == 0.0) {
//			printf("time: %lf aps list size: %d\n", time_, listAPS_.size());
//			int i=0;
//			for (APSIterator it=listAPS_.begin(); it!=listAPS_.end(); it++, i++) {
//				printf("APS %d: (%.2lf | %.2lf) (%.2lf | %.2lf) (%.2lf | %.2lf) (%.2lf | %.2lf) <-",i, it->getTimeFootTouchDown(0),it->getTimeFootLiftOff(0),
//																			   it->getTimeFootTouchDown(1),it->getTimeFootLiftOff(1),
//																			   it->getTimeFootTouchDown(2),it->getTimeFootLiftOff(2),
//																			   it->getTimeFootTouchDown(3),it->getTimeFootLiftOff(3)
//																			  );
//				for (int iLeg=0; iLeg<4; iLeg++) {
//					if (currentAPS[iLeg] == it) {
//						printf("\e[32m%dc \e[0m", iLeg);
//					} else if(nextAPS[iLeg] == it) {
//						printf("\e[33m%dn \e[0m", iLeg);
//					} else if(prevAPS[iLeg] == it) {
//						printf("\e[95m%dp \e[0m", iLeg);
//					} else if(nextNextAPS[iLeg] == it) {
//						printf("\e[93m%df \e[0m", iLeg);
//					}
//
//				}
//				printf("\n");
//			}
//		}
		/* end debug */

		if (time_ > currentAPSStanceTimeEnd) {

			/* stance phase is over, but APS of current leg has not yet finished */
			if (currentAPS[iLeg] == currentAPS[0]) {
				/* new APS of first leg has not yet started */
				stancePhases_[iLeg] = 0.0;
				const double swingDuration = nextAPSStanceTimeStart-currentAPSStanceTimeEnd;
				if (swingDuration != 0) {
					swingPhases_[iLeg] = (time_-currentAPSStanceTimeEnd)/(swingDuration);
				} else {
					swingPhases_[iLeg] = -1.0;
				}



			} else {
				/* new APS of first leg has started */
				currentAPS[iLeg]++;
				prevAPS[iLeg]++;
				nextAPS[iLeg]++;
				nextNextAPS[iLeg] = nextAPS[iLeg];
				nextNextAPS[iLeg]++;
				if (nextNextAPS[iLeg] == listAPS_.end()) {
					nextNextAPS[iLeg] = nextAPS[iLeg];
				}


				if (time_ >= nextAPSStanceTimeStart && time_ <= nextAPSStanceTimeEnd) {
					/* leg is in stance mode in new APS */
					const double stanceDuration = nextAPSStanceTimeEnd-nextAPSStanceTimeStart;
					if (stanceDuration != 0.0) {
						stancePhases_[iLeg] = (time_-nextAPSStanceTimeStart)/(stanceDuration);
					} else {
						stancePhases_[iLeg] = 0.0;
					}
					if (stancePhases_[iLeg] > 1 || stancePhases_[iLeg] < 0) {
						stancePhases_[iLeg] = 0.0;
						const double swingDuration = nextAPSStanceTimeStart-currentAPSStanceTimeEnd;
						if (swingDuration != 0) {
							swingPhases_[iLeg] = (time_-currentAPSStanceTimeEnd)/(swingDuration);
						} else {
							swingPhases_[iLeg] = -1.0;
						}
					} else {
						swingPhases_[iLeg] = -1.0;
					}
				} else {
					/* leg is not yet in stance mode in new APS */
					stancePhases_[iLeg] = 0.0;
					const double swingDuration = nextAPSStanceTimeStart-currentAPSStanceTimeEnd;
					if (swingDuration != 0.0) {
						swingPhases_[iLeg] = (time_-currentAPSStanceTimeEnd)/(swingDuration);
					} else {
						swingPhases_[iLeg] = -1.0;
					}
				}
			}
		} else {
			/* current APS is active */

			if (time_ >= currentAPSStanceTimeStart && time_ <= currentAPSStanceTimeEnd) {
				/* leg is in stance mode in new APS */
				const double stanceDuration = currentAPSStanceTimeEnd-currentAPSStanceTimeStart;
				if (stanceDuration != 0.0) {
					stancePhases_[iLeg] = (time_-currentAPSStanceTimeStart)/(stanceDuration);
				} else {
					stancePhases_[iLeg] = 0.0;
				}
				if (stancePhases_[iLeg] > 1 || stancePhases_[iLeg] < 0) {
					stancePhases_[iLeg] = 0.0;
					const double swingDuration = nextAPSStanceTimeStart-currentAPSStanceTimeEnd;
					if (swingDuration != 0.0) {
						swingPhases_[iLeg] = (time_-currentAPSStanceTimeEnd)/(swingDuration);
					} else {
						swingPhases_[iLeg] = -1.0;
					}
				} else {
					swingPhases_[iLeg] = -1.0;
				}
			} else {
				/* leg is not yet in stance mode in new APS */
				stancePhases_[iLeg] = 0.0;
				const double swingDuration = currentAPSStanceTimeStart-prevAPSStanceTimeEnd;
				if (swingDuration != 0.0) {
					swingPhases_[iLeg] = (time_-prevAPSStanceTimeEnd)/(swingDuration);
				} else {
					swingPhases_[iLeg] = -1.0;
				}

			}
		}

	}


}


double GaitAPS::getTime() {
	return time_;
}





void GaitAPS::printTDandLO()
{

			printf("time: %lf aps list size: %d\n", time_, (int)listAPS_.size());
			int i=0;
			for (APSIterator it=listAPS_.begin(); it!=listAPS_.end(); it++, i++) {
				printf("APS %d: TD: %.2lf %.2lf %.2lf %.2lf LO: %.2lf %.2lf %.2lf %.2lf <-",i, it->getTimeFootTouchDown(0),
																				   it->getTimeFootTouchDown(1),
																				   it->getTimeFootTouchDown(2),
																				   it->getTimeFootTouchDown(3),
																				   it->getTimeFootLiftOff(0),
																				   it->getTimeFootLiftOff(1),
																				   it->getTimeFootLiftOff(2),
																				   it->getTimeFootLiftOff(3));
				for (int iLeg=0; iLeg<4; iLeg++) {
					if (currentAPS[iLeg] == it) {
						printf("\e[32m%d \e[0m", iLeg);
					} else if(nextAPS[iLeg] == it) {
						printf("\e[33m%d \e[0m", iLeg);
					} else if(prevAPS[iLeg] == it) {
						printf("\e[95m%d \e[0m", iLeg);
					} else if(nextNextAPS[iLeg] == it) {
						printf("\e[93m%d \e[0m", iLeg);
					}

				}
				printf("\n");
			}

}

APS* GaitAPS::getLastAPS()
{
	return &listAPS_.back();
}

APS* GaitAPS::getFirstAPS()
{
	return &listAPS_.front();
}

void GaitAPS::addAPS(APS aps)
{
	aps.startTime_ = listAPS_.back().getTimeAPSEnd();
	listAPS_.push_back(aps);
}
void GaitAPS::print() {
	printf("time: %lf aps list size: %d\n", time_, (int)listAPS_.size());
	int i=0;
	for (APSIterator it=listAPS_.begin(); it!=listAPS_.end(); it++, i++) {
//		printf("APS %d: startTime: %lf phase: %lf\n",i, it->startTime_, it->phase_);
		printf("--------- APS #%d ---------  \n", i);
		printf("legs: ");
		for (int i=0;i<4;i++) {
			if (it==currentAPS[i]) {
				printf("%d ",i);
			}
		}
		printf("\n");
//		printf("phase: %lf\n", it->phase_);
//		printf("startTime: %lf\n", it->startTime_);
//		printf("cycleDuration: %lf\n",it->cycleDuration_);
//		printf("foreDutyFactor: %lf\n", it->foreDutyFactor_);
//		printf("hindDutyFactor: %lf\n", it->hindDutyFactor_);
//		printf("foreLag: %lf\n", it->foreLag_);
//		printf("hindLag: %lf\n", it->hindLag_);
//		printf("pairLag: %lf\n", it->pairLag_);
//		printf("interpolate: %lf\n", it->interpolate_);
		it->print();
	}


}

APS* GaitAPS::getCurrentAPS() {
	return &(*currentAPS[0]);
}

APS* GaitAPS::getNextAPS() {
	return &(*nextAPS[0]);
}


void GaitAPS::popLastAPS()
{
	listAPS_.pop_back();
}

APS* GaitAPS::getCurrentAPS(int iLeg)
{
	return &(*currentAPS[iLeg]);
}
APS* GaitAPS::getNextAPS(int iLeg)
{
	return &(*nextAPS[iLeg]);
}

APS* GaitAPS::getNextNextAPS(int iLeg)
{
	return &(*nextNextAPS[iLeg]);
}
APS* GaitAPS::getPreviousAPS(int iLeg)
{
	return &(*prevAPS[iLeg]);
}
APS* GaitAPS::getPreviousPreviousAPS(int iLeg)
{
	std::list<APS>::iterator it =  prevAPS[iLeg];

	if (it != listAPS_.begin()) {
		it--;
	}

	return &(*it);
}


double GaitAPS::getSwingPhase(int iLeg)
{
	return swingPhases_[iLeg];
}

double GaitAPS::getStancePhase(int iLeg)
{
	return stancePhases_[iLeg];
}

double GaitAPS::getStanceDuration(int iLeg)
{
	return currentAPS[iLeg]->getStanceDurationForLeg(iLeg);
}

double GaitAPS::getSwingDuration(int iLeg)
{
	return nextAPS[iLeg]->getTimeFootTouchDown(iLeg)-currentAPS[iLeg]->getTimeFootLiftOff(iLeg);
//	return currentAPS[iLeg]->cycleDuration_-getStanceDuration(iLeg);
}


unsigned long int GaitAPS::getNumGaitCycles()
{
	return numGaitCycles;
}


GaitAPS::APSIterator GaitAPS::getIteratorBegin()
{
	return listAPS_.begin();
}
GaitAPS::APSIterator GaitAPS::getIteratorEnd()
{
	return listAPS_.end();
}
bool GaitAPS::shouldBeGrounded(int iLeg)
{
	return swingPhases_[iLeg]==-1;
}

unsigned int  GaitAPS::getAPSSize()
{
	return listAPS_.size();
}

void GaitAPS::resetInterpolation()
{
	for (int iLeg =0; iLeg<4; iLeg++) {
		currentAPS[iLeg]->interpolate_ = 0.0;
		nextAPS[iLeg]->interpolate_ = 0.0;
		nextNextAPS[iLeg]->interpolate_ = 0.0;
	}
}

} // namespace loco

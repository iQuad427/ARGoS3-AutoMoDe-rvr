/**
 * @file <src/modules/AutoMoDeConditionRedFloor.h>
 *
 * @author Raffaele Todesco - <raffaele.todesco@ulb.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#ifndef AUTOMODE_CONDITION_BLACK_FLOOR_H
#define AUTOMODE_CONDITION_BLACK_FLOOR_H

#include "AutoMoDeCondition.h"

namespace argos
{
	class AutoMoDeConditionRedFloor : public AutoMoDeCondition
	{
	public:
		AutoMoDeConditionRedFloor();
		virtual ~AutoMoDeConditionRedFloor();

		AutoMoDeConditionRedFloor(AutoMoDeConditionRedFloor *pc_condition);
		virtual AutoMoDeConditionRedFloor *Clone();

		virtual bool Verify();
		virtual void Reset();
		virtual void Init();

	private:
		const CColor m_cReferenceColor = CColor::RED;
		// max deltaE* between the color and reference
		Real m_fColorThreshold;
		Real m_fProbability;
	};
}

#endif

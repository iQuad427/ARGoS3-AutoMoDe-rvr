/**
 * @file <src/modules/AutoMoDeConditionInvertedLightIntensity.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#ifndef AUTOMODE_CONDITION_INVERTED_LIGHT_INTENSITY_H
#define AUTOMODE_CONDITION_INVERTED_LIGHT_INTENSITY_H

#include "AutoMoDeCondition.h"

namespace argos
{
	class AutoMoDeConditionInvertedLightIntensity : public AutoMoDeCondition
	{
	public:
		AutoMoDeConditionInvertedLightIntensity();
		virtual ~AutoMoDeConditionInvertedLightIntensity();

		AutoMoDeConditionInvertedLightIntensity(AutoMoDeConditionInvertedLightIntensity *pc_condition);
		virtual AutoMoDeConditionInvertedLightIntensity *Clone();

		virtual bool Verify();
		virtual void Reset();
		virtual void Init();

	private:
		Real m_fParameterEta;
		UInt8 m_unParameterXi;
	};
}

#endif

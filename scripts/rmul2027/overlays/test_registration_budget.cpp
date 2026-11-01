// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "small_gicp_relocalization/registration_budget.hpp"
using small_gicp_relocalization::checkedRegistrationBudget;
TEST(RegistrationBudget, LegacyDefaultIsUnchanged) {EXPECT_EQ(checkedRegistrationBudget(10),10);}
TEST(RegistrationBudget, ExplicitCadBudgetIsFinite) {EXPECT_EQ(checkedRegistrationBudget(32),32);EXPECT_EQ(checkedRegistrationBudget(64),64);}
TEST(RegistrationBudget, InvalidOrUnboundedValuesAreRejected) {
 for(int value : {-1,0,65,std::numeric_limits<int>::max()}) {EXPECT_THROW(checkedRegistrationBudget(value),std::invalid_argument);}
}
TEST(RegistrationBudget, EntireAllowedRangeIsBounded) {
 for(int value=1;value<=64;++value) {EXPECT_EQ(checkedRegistrationBudget(value),value);}
}

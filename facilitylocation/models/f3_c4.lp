\ Model FacilityLocation
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  100 open0 + 100 open1 + 100 open2
   + 469.5189027078675 cust_to_facility_f0_c_0
   + 473.7615433949869 cust_to_facility_f1_c_0
   + 1975.656346635214 cust_to_facility_f2_c_0
   + 470.9331162702406 cust_to_facility_f0_c_1
   + 475.1757569573599 cust_to_facility_f1_c_1
   + 1977.070560197587 cust_to_facility_f2_c_1
   + 472.3473298326138 cust_to_facility_f0_c_2
   + 476.589970519733 cust_to_facility_f1_c_2
   + 1978.48477375996 cust_to_facility_f2_c_2
   + 677.4082963767125 cust_to_facility_f0_c_3
   + 673.1656556895932 cust_to_facility_f1_c_3
   + 828.7291475506337 cust_to_facility_f2_c_3
Subject To
 one_cust_per_facility_c0: cust_to_facility_f0_c_0
   + cust_to_facility_f1_c_0 + cust_to_facility_f2_c_0 = 1
 one_cust_per_facility_c1: cust_to_facility_f0_c_1
   + cust_to_facility_f1_c_1 + cust_to_facility_f2_c_1 = 1
 one_cust_per_facility_c2: cust_to_facility_f0_c_2
   + cust_to_facility_f1_c_2 + cust_to_facility_f2_c_2 = 1
 one_cust_per_facility_c3: cust_to_facility_f0_c_3
   + cust_to_facility_f1_c_3 + cust_to_facility_f2_c_3 = 1
 facility_capacity_f0: - 100 open0 + 50 cust_to_facility_f0_c_0
   + 50 cust_to_facility_f0_c_1 + 75 cust_to_facility_f0_c_2
   + 75 cust_to_facility_f0_c_3 <= 0
 facility_capacity_f1: - 100 open1 + 50 cust_to_facility_f1_c_0
   + 50 cust_to_facility_f1_c_1 + 75 cust_to_facility_f1_c_2
   + 75 cust_to_facility_f1_c_3 <= 0
 facility_capacity_f2: - 500 open2 + 50 cust_to_facility_f2_c_0
   + 50 cust_to_facility_f2_c_1 + 75 cust_to_facility_f2_c_2
   + 75 cust_to_facility_f2_c_3 <= 0
Bounds
Binaries
 open0 open1 open2 cust_to_facility_f0_c_0 cust_to_facility_f1_c_0
 cust_to_facility_f2_c_0 cust_to_facility_f0_c_1 cust_to_facility_f1_c_1
 cust_to_facility_f2_c_1 cust_to_facility_f0_c_2 cust_to_facility_f1_c_2
 cust_to_facility_f2_c_2 cust_to_facility_f0_c_3 cust_to_facility_f1_c_3
 cust_to_facility_f2_c_3
End

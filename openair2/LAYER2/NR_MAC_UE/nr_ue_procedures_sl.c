/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

#include "mac_defs.h"
#include "mac_proto.h"
#include "openair2/LAYER2/NR_MAC_COMMON/nr_mac_common.h"
#include "executables/softmodem-common.h"

#define SL_DEBUG

static const int sequence_cyclic_shift_harq_ack_or_ack_or_only_nack[2]
/* Sequence cyclic shift */ = {  0, 6 };

typedef struct prbs_set {
  uint16_t **start_prb;
  uint16_t **end_prb;
} prbs_set_t;

typedef struct psfch_params {
  uint16_t m0;
  prbs_set_t *prbs_sets;
} psfch_params_t;

void print_prb_set_allocation(psfch_params_t *psfch_params, uint8_t psfch_period, uint8_t  num_subchannels) {
  LOG_D(NR_PHY, "PSSCH Slot mod PSFCH period |   Subchannel   |   Start PRB   |    End PRB\n");
  for (int i = 0; i < psfch_period; i++) {
    for (int j = 0; j < num_subchannels; j++) {
      LOG_D(NR_PHY, "\t\t    %d \t\t|\t%d\t|\t%d\t| \t %d\n", i, j, psfch_params->prbs_sets->start_prb[i][j], psfch_params->prbs_sets->end_prb[i][j]);
    }
  }
}

uint8_t sl_process_TDD_UL_DL_config_patterns(NR_TDD_UL_DL_ConfigCommon_t *TDD_UL_DL_Config,
                                             uint8_t mu,
                                             double *slot_period_P,
                                             uint8_t *w)
{

  uint8_t return_value = 255;
  *w = 0;
  int pattern1_dlul_period = TDD_UL_DL_Config->pattern1.dl_UL_TransmissionPeriodicity;

#ifdef SL_DEBUG

  printf("INPUT VALUES: function: %s\n", __func__);
  printf("pattern1 periodicity:%d\n", pattern1_dlul_period);
  if (TDD_UL_DL_Config->pattern1.ext1 != NULL && TDD_UL_DL_Config->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530 != NULL )
    printf("pattern1 periodicity_v1530:%ld\n", *TDD_UL_DL_Config->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530);
  if (TDD_UL_DL_Config->pattern2 != NULL) {
    printf("mu:%d, pattern2 periodicity:%d\n", mu, pattern1_dlul_period);
    if (TDD_UL_DL_Config->pattern2->ext1 != NULL && TDD_UL_DL_Config->pattern2->ext1->dl_UL_TransmissionPeriodicity_v1530 != NULL )
      printf("pattern2 periodicity_v1530:%ld\n", *TDD_UL_DL_Config->pattern2->ext1->dl_UL_TransmissionPeriodicity_v1530);
  }

#endif

  return_value = pattern1_dlul_period;
  switch (pattern1_dlul_period) {
    case 0:
      *slot_period_P = 0.5;
      break;
    case 1:
      *slot_period_P = 0.625;
      break;
    case 2:
      *slot_period_P = 1.0;
      break;
    case 3:
      *slot_period_P = 1.25;
      break;
    case 4:
      *slot_period_P = 2.0;
      break;
    case 5:
      *slot_period_P = 2.5;
      break;
    case 6:
      *slot_period_P = 5.0;
      return_value = 7;
      break;
    case 7:
      *slot_period_P = 10.0;
      return_value = 8;
      break;
    default:
      AssertFatal(1==0,"Incorrect value of dl_UL_TransmissionPeriodicity\n");
      break;
  }

  if (TDD_UL_DL_Config->pattern1.ext1 != NULL &&
      TDD_UL_DL_Config->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530 != NULL ) {
    if (*TDD_UL_DL_Config->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530 == 1) {
      *slot_period_P = 4.0;
      return_value = 6;
    } else {
      *slot_period_P = 3.0;
      return_value = 255;
    }
  }

  if (TDD_UL_DL_Config->pattern2 != NULL) {

    return_value = 255;
    *w = 1;

    if ((*slot_period_P == 4.0 ) && (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 1)) {
      return_value = 13;
      *w = (mu == 3)? 2: 1;
    } else if ((*slot_period_P == 3.0 ) && (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 4)) {
      return_value = 12;
      *w = (mu == 3)? 2: 1;
    } else if ((*slot_period_P == 3.0 ) && (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 2)) {
      return_value = 8;
      *w = (mu == 3)? 2: 1;
    } else {

      switch (pattern1_dlul_period) {
        case 7:
          if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 7) {
            return_value = 15;
            *w = 1<<mu;
          }
          break;
        case 6:
          if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 6) {
            return_value = 14;
            *w = (mu==0)?1:1<<(mu-1);
          }
          break;
        case 5:
          if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 5) {
            return_value = 11;
            *w = (mu == 3)? 2: 1;
          }
          break;
        case 4:
          if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 0) {
            return_value = 5;
          }
          else if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 4) {
            return_value = 7;
          }
          else if (TDD_UL_DL_Config->pattern2->ext1 != NULL && *TDD_UL_DL_Config->pattern2->ext1->dl_UL_TransmissionPeriodicity_v1530 == 0) {
            return_value = 10;
            *w = (mu == 3)? 2: 1;
          }
          break;
        case 3:
          if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 3) {
            return_value = 4;
          }
          break;
        case 2:
          if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 2) {
            return_value = 2;
          }
          else if (TDD_UL_DL_Config->pattern2->ext1 != NULL && *TDD_UL_DL_Config->pattern2->ext1->dl_UL_TransmissionPeriodicity_v1530 == 0) {
            return_value = 6;
            *w = (mu == 3)? 2: 1;
          }
          else if (TDD_UL_DL_Config->pattern2->ext1 != NULL && *TDD_UL_DL_Config->pattern2->ext1->dl_UL_TransmissionPeriodicity_v1530 == 1) {
            return_value = 9;
            *w = (mu == 3)? 2: 1;
          }
          break;
        case 1:
          if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 1) {
            return_value = 1;
          }
          break;
        case 0:
          if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 0) {
            return_value = 0;
          }
          else if (TDD_UL_DL_Config->pattern2->dl_UL_TransmissionPeriodicity == 4) {
            return_value = 3;
          }
          break;
        default:
          AssertFatal(1==0,"Incorrect value of dl_UL_TransmissionPeriodicity");
      }
    }
  }

#ifdef SL_DEBUG
  printf("OUTPUT VALUES: function %s\n",__func__);
  printf("return_value:%d, *w:%d, slot_period_P:%f\n", return_value, *w, *slot_period_P);
#endif

  return return_value;
}

/*
This procedures prepares the psbch payload of tdd configuration according
to section 16.1 in 38.213
*/
void sl_prepare_psbch_payload(NR_TDD_UL_DL_ConfigCommon_t *TDD_UL_DL_Config,
                              uint8_t *bits_0_to_7, uint8_t *bits_8_to_11,
                              uint8_t mu, uint8_t L, uint8_t Y)
{

  uint8_t w = 0, a1_to_a4 = 0;
  uint8_t mu_ref = 0, diff = 0;
  uint8_t u_slots = 0, u_sym = 0, I1 = 0;
  uint8_t u_sl_slots = 0, u_sl_slots_2 = 0;
  double slot_period_P = 0.0;

  *bits_0_to_7  = 0xFF; // If TDD_UL_DL_Config = NULL all 12 bits are set to 1
  *bits_8_to_11 = 0xF0;

  if (TDD_UL_DL_Config != NULL) {

    mu_ref = TDD_UL_DL_Config->referenceSubcarrierSpacing;
    diff = 1 << (mu-mu_ref);
    u_slots = TDD_UL_DL_Config->pattern1.nrofUplinkSlots;
    u_sym = TDD_UL_DL_Config->pattern1.nrofUplinkSymbols;
    I1 = ((u_sym * diff) % L >= (L-Y)) ? 1 : 0;

#ifdef SL_DEBUG
    printf("INPUT VALUES: function %s\n", __func__);
    printf("numerology:%d, number of symbols:%d, sl-startSymbol:%d\n", mu, L, Y);
    printf("mu_ref:%d, u_slots:%d, u_sym:%d\n", mu_ref, u_slots, u_sym);
    if (TDD_UL_DL_Config->pattern2 != NULL)
      printf("u_slots_2:%ld, u_sym_2:%ld\n", TDD_UL_DL_Config->pattern2->nrofUplinkSlots,
                                             TDD_UL_DL_Config->pattern2->nrofUplinkSymbols);
#endif

    u_sl_slots = (u_slots * diff) + floor((u_sym*diff)/L) + I1;
    a1_to_a4 = sl_process_TDD_UL_DL_config_patterns(TDD_UL_DL_Config, mu, &slot_period_P, &w);
    AssertFatal(a1_to_a4 != 255,"Incorrect return value, wrong configuration.\n");

#ifdef SL_DEBUG
    printf("I1:%d, a1_to_a2:%d, u_sl_slots:%d\n", I1, a1_to_a4, u_sl_slots);
#endif

    if (TDD_UL_DL_Config->pattern2 != NULL) {

      uint8_t u_slots_2 = TDD_UL_DL_Config->pattern2->nrofUplinkSlots;
      uint8_t u_sym_2 = TDD_UL_DL_Config->pattern2->nrofUplinkSymbols;
      uint8_t I2 = ((u_sym_2 * diff) % L >= (L-Y)) ? 1 : 0;
      uint16_t val = floor(((u_slots_2 * diff) + floor((u_sym_2*diff)/L) + I2)/w);

      u_sl_slots_2 = val * ceil((slot_period_P*(1<<mu)+1)/w) + floor(u_sl_slots/w);

      *bits_0_to_7 = 0x80 | (a1_to_a4 << 3) | ((u_sl_slots_2 & 0x70) >> 4);
      *bits_8_to_11 = (u_sl_slots_2 & 0x0F) << 4;

#ifdef SL_DEBUG
    printf("I2:%d, val:%d, u_sl_slots_2:%d\n", I2, val, u_sl_slots_2);
#endif

    } else {
      *bits_0_to_7 = 0x00 | (a1_to_a4 << 3) | ((u_sl_slots & 0x70) >> 4);
      *bits_8_to_11 = (u_sl_slots & 0x0F) << 4;
    }
  }

#ifdef SL_DEBUG
    printf("OUTPUT VALUES: function %s\n", __func__);
    printf("12 bits payload buf[0]:%x, buf[1]:%x\n", *bits_0_to_7, *bits_8_to_11);
#endif

}

/*
This procedures prepares the psbch payload of tdd configuration according
to section 16.1 in 38.213
*/
uint8_t sl_decode_sl_TDD_Config(NR_TDD_UL_DL_ConfigCommon_t *TDD_UL_DL_Config,
                                uint8_t bits_0_to_7, uint8_t bits_8_to_11,
                                uint8_t mu, uint8_t L, uint8_t Y)
{

  AssertFatal(TDD_UL_DL_Config, "TDD_UL_DL_Config cannot be null");
  uint16_t num_SL_slots = 0, mixed_slot_numsym = 0;

  TDD_UL_DL_Config->pattern1.nrofDownlinkSlots = 0;
  TDD_UL_DL_Config->pattern1.nrofDownlinkSymbols = 0;
  TDD_UL_DL_Config->pattern1.nrofUplinkSlots = 0;
  TDD_UL_DL_Config->pattern1.nrofUplinkSymbols = 0;
  TDD_UL_DL_Config->referenceSubcarrierSpacing = mu;
  TDD_UL_DL_Config->pattern1.ext1 = NULL;

  LOG_D(MAC, "bits_0_to_7:%x, bits_8_to_11:%x, mu:%d, L:%d, Y:%d\n",
                                                  bits_0_to_7, bits_8_to_11,mu, L, Y);

  //If all bits are 1 - indicates that no TDD config was present.
  if ((bits_0_to_7 == 0xFF) && ((bits_8_to_11 & 0xF0) == 0xF0)) {
    //If no TDD config present - use all slots for Sidelink.
    //Spec not clear -- TBD....
    return 0;
  }

  //Bit A0 if 1 indicates pattern2 as present.
  if (bits_0_to_7 & 0x80) {
    //Pattern1 and Pattern2 Present.
    TDD_UL_DL_Config->pattern2 = malloc16_clear(sizeof(*TDD_UL_DL_Config->pattern2));
    AssertFatal(1==0,"Decoding Pattern2 - NOT YET IMPLEMENTED\n");
  } else {

    //Only Pattern1 Present. bits a1..a4 identify the periodicity.
    uint8_t val = (bits_0_to_7 & 0x78) >> 3;
    if (val >= 7)
      TDD_UL_DL_Config->pattern1.dl_UL_TransmissionPeriodicity = val-1;

    if (val == 6) {
      if (TDD_UL_DL_Config->pattern1.ext1 == NULL)
        TDD_UL_DL_Config->pattern1.ext1 = calloc(1, sizeof(*TDD_UL_DL_Config->pattern1.ext1));
      if (TDD_UL_DL_Config->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530 == NULL)
        TDD_UL_DL_Config->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530 = calloc(1, sizeof(long));
      *TDD_UL_DL_Config->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530 = 1;
    }

    //a5,a6..a11 bits from the 7th to 1st LSB of num SL slots
    num_SL_slots = ((bits_0_to_7 & 0x07) << 4 ) | ((bits_8_to_11 & 0xF0) >> 4);

    TDD_UL_DL_Config->pattern1.nrofUplinkSlots = num_SL_slots;
    TDD_UL_DL_Config->pattern1.nrofUplinkSymbols = mixed_slot_numsym;

    LOG_D(MAC, "SIDELINK: EXtracted TDD config from 12 bits - Sidelink Slots:%ld, Mixed_slot_symbols:%ld,dl_UL_TransmissionPeriodicity:%ld\n",
                                TDD_UL_DL_Config->pattern1.nrofUplinkSlots, TDD_UL_DL_Config->pattern1.nrofUplinkSymbols,
                                TDD_UL_DL_Config->pattern1.dl_UL_TransmissionPeriodicity);
  }
  return 1;
}

/*Function used to prepare Sidelink MIB*/
uint32_t sl_prepare_MIB(NR_TDD_UL_DL_ConfigCommon_t *TDD_UL_DL_Config,
                        uint8_t incoverage, uint8_t mu,
                        uint8_t start_symbol, uint8_t L) {

  uint8_t  sl_mib_payload[4] = {0,0,0,0};
  //int mu = UE->sl_frame_params.numerology_index, start_symbol = UE->start_symbol;
  uint8_t byte0, byte1;
  //int L = (UE->sl_frame_params.Ncp == 0) ? 14 : 12;
  uint32_t sl_mib=0;

  sl_prepare_psbch_payload(TDD_UL_DL_Config, &byte0, &byte1, mu, L, start_symbol);
  sl_mib_payload[0] = byte0;
  sl_mib_payload[1] = byte1;

  AssertFatal(incoverage <= 1, "Invalid value for incoverage paramter for SL-MIB. Accepted values 0 or 1\n");
  sl_mib_payload[1] |= (incoverage << 3);

  sl_mib =  sl_mib_payload[1]<<8  | sl_mib_payload[0];

#ifdef SL_DEBUG
  printf("SIDELINK PSBCH SIM: NUM SYMBOLS:%d, mu:%d, start_symbol:%d incoverage:%d \n",
                                      L, mu, start_symbol, incoverage);
  printf("SIDELINK PSBCH PAYLOAD: psbch_a:%x, sl_mib_payload:%x %x %x %x\n",
                                sl_mib, sl_mib_payload[0],sl_mib_payload[1], sl_mib_payload[2], sl_mib_payload[3]);
#endif

  return sl_mib;
}

uint16_t sl_get_subchannel_size(NR_SL_ResourcePool_r16_t *rpool)
{

  uint16_t subch_size = 0;
  const uint8_t subchsizes[8] = {10, 12, 15, 20, 25, 50, 75, 100};

  subch_size = (rpool->sl_SubchannelSize_r16)
                   ? subchsizes[*rpool->sl_SubchannelSize_r16] : 0;

  AssertFatal(subch_size,"Subch Size cannot be 0.Resource Pool Configuration Error\n");

  return subch_size;
}

uint16_t sl_get_num_subch(NR_SL_ResourcePool_r16_t *rpool)
{

  uint16_t num_subch = 0;
  uint16_t subch_size = sl_get_subchannel_size(rpool);
  uint16_t num_rbs = (rpool->sl_RB_Number_r16) ? *rpool->sl_RB_Number_r16 : 0;

  AssertFatal(num_rbs,"NumRbs in rpool cannot be 0.Resource Pool Configuration Error\n");

  num_subch = num_rbs/subch_size;

  LOG_I(NR_MAC, "Subch_size:%d, numRBS:%d, num_subch:%d\n",
                                          subch_size,num_rbs,num_subch);

  return (num_subch);
}

//This function determines SCI 1A Len in bits based on the configuration in the resource pool.
uint8_t sl_determine_sci_1a_len(uint16_t *num_subchannels,
                                NR_SL_ResourcePool_r16_t *rpool,
                                sidelink_sci_format_1a_fields_t *sci_1a)
{

  uint8_t num_bits = 0;

  //Size of Fixed fields prio (3), sci_2ndstage(2),
  //betaoffsetindicator(2), num dmrs ports (1), mcs (5bits)
  uint8_t sci_1a_len = SL_SCI_FORMAT_1A_LEN_IN_BITS_FIXED_FIELDS;

  *num_subchannels = sl_get_num_subch(rpool);

  uint16_t n_subch = *num_subchannels;

  LOG_D(NR_MAC,"Determine SCI-1A len - Num Subch:%d, sci 1A len fixed fields:%d\n",
                                                           *num_subchannels, sci_1a_len);

  NR_SL_UE_SelectedConfigRP_r16_t *selectedconfigRP = rpool->sl_UE_SelectedConfigRP_r16;
  const uint8_t maxnum_values[] = {2,3};
  uint8_t sl_MaxNumPerReserve =   (selectedconfigRP &&
                                   selectedconfigRP->sl_MaxNumPerReserve_r16)
                                   ? maxnum_values[*selectedconfigRP->sl_MaxNumPerReserve_r16]
                                   : 0;

  //Determine bits for Freq and Time Resource assignment
  if (sl_MaxNumPerReserve == 3) {
    num_bits = ceil(log2(n_subch * (n_subch + 1) * (2*n_subch + 1)/6));
    sci_1a_len += num_bits;
    sci_1a->frequency_resource_assignment.nbits = num_bits;
    sci_1a_len += 9;
    sci_1a->time_resource_assignment.nbits = 9;
  } else {
    num_bits = ceil(log2((n_subch * (n_subch + 1)) >> 1));
    sci_1a_len += num_bits;
    sci_1a->frequency_resource_assignment.nbits = num_bits;
    sci_1a_len += 5;
    sci_1a->time_resource_assignment.nbits = 5;
  }

  LOG_D(NR_MAC,"sci 1A - sl_MaxNumPerReserve:%d, sci 1a len:%d, FRA nbits:%d, TRA nbits:%d\n",
                                                                    sl_MaxNumPerReserve,sci_1a_len,
                                                                    sci_1a->frequency_resource_assignment.nbits,
                                                                    sci_1a->time_resource_assignment.nbits);

  //Determine bits for res reservation period
  uint8_t n_rsvperiod =  (selectedconfigRP &&
                          selectedconfigRP->sl_ResourceReservePeriodList_r16)
                          ? selectedconfigRP->sl_ResourceReservePeriodList_r16->list.count : 0;

  #define SL_IE_ENABLED 0
  if (selectedconfigRP &&
      selectedconfigRP->sl_MultiReserveResource_r16 == SL_IE_ENABLED) {
    num_bits = ceil(log2(n_rsvperiod));
    sci_1a_len += num_bits;
    sci_1a->resource_reservation_period.nbits = num_bits;
  } else
    sci_1a->resource_reservation_period.nbits = 0;

  LOG_D(NR_MAC,"sci 1A - n_rsvperiod:%d, sci 1a len:%d, res reserve period.nbits:%d\n",
                                                      n_rsvperiod, sci_1a_len,
                                                      sci_1a->resource_reservation_period.nbits);


  uint8_t n_dmrspatterns = 0;
  if (rpool->sl_PSSCH_Config_r16 &&
      rpool->sl_PSSCH_Config_r16->present == NR_SetupRelease_SL_PSSCH_Config_r16_PR_setup) {
    NR_SL_PSSCH_Config_r16_t *pssch_cfg = rpool->sl_PSSCH_Config_r16->choice.setup;

    //Determine bits for DMRS PATTERNS
    n_dmrspatterns = (pssch_cfg && pssch_cfg->sl_PSSCH_DMRS_TimePatternList_r16)
                         ? pssch_cfg->sl_PSSCH_DMRS_TimePatternList_r16->list.count : 0;
  }

  AssertFatal((n_dmrspatterns>=1) && (n_dmrspatterns <=3),
                          "Number of DMRS Patterns should be 1or2or3. Resource Pool Configuration Error.\n");

  if (n_dmrspatterns) {
    num_bits = ceil(log2(n_dmrspatterns));
    sci_1a_len += num_bits;
    sci_1a->dmrs_pattern.nbits = num_bits;
  }

  LOG_D(NR_MAC,"sci 1A -  n_dmrspatterns:%d, sci 1a len:%d, dmrs_pattern.nbits:%d\n",
                                                  n_dmrspatterns, sci_1a_len, sci_1a->dmrs_pattern.nbits);

  //Determine bits for Additional MCS table
  if (rpool->sl_Additional_MCS_Table_r16) {
    int numbits = (*rpool->sl_Additional_MCS_Table_r16 > 1) ? 2 : 1;
    sci_1a_len += numbits;
    sci_1a->additional_mcs_table_indicator.nbits = numbits;
    AssertFatal(*rpool->sl_Additional_MCS_Table_r16<=2, "additional table value cannot be > 2. Resource Pool Configuration Error.\n");
  }

  LOG_D(NR_MAC,"sci 1A - additional_table:%ld, sci 1a len:%d, additional table nbits:%d\n",
                                                                rpool->sl_Additional_MCS_Table_r16 ? *rpool->sl_Additional_MCS_Table_r16 : 0,
                                                                sci_1a_len,
                                                                sci_1a->additional_mcs_table_indicator.nbits);

  uint8_t psfch_period = 0;
  if (rpool->sl_PSFCH_Config_r16 &&
      rpool->sl_PSFCH_Config_r16->present == NR_SetupRelease_SL_PSFCH_Config_r16_PR_setup) {
    NR_SL_PSFCH_Config_r16_t *psfch_config = rpool->sl_PSFCH_Config_r16->choice.setup;

    //Determine bits for PSFCH overhead indication
    const uint8_t psfch_periods[] = {0,1,2,4};
    psfch_period = (psfch_config->sl_PSFCH_Period_r16)
                          ? psfch_periods[*psfch_config->sl_PSFCH_Period_r16] : 0;
  }

  if ((psfch_period == 2) || (psfch_period == 4)) {
    sci_1a_len += 1;
    sci_1a->psfch_overhead_indication.nbits = 1;
  } else
    sci_1a->psfch_overhead_indication.nbits = 0;

  LOG_D(NR_MAC,"sci 1A - psfch_period:%d, sci 1a len:%d, psfch overhead nbits:%d\n",
                                                            psfch_period, sci_1a_len,
                                                            sci_1a->psfch_overhead_indication.nbits);

  //Determine number of reserved bits
  uint8_t num_reservedbits =  0;
  if (rpool->sl_PSCCH_Config_r16 &&
      rpool->sl_PSCCH_Config_r16->present == NR_SetupRelease_SL_PSCCH_Config_r16_PR_setup) {
    NR_SL_PSCCH_Config_r16_t *pscch_config = rpool->sl_PSCCH_Config_r16->choice.setup;

    num_reservedbits = (pscch_config->sl_NumReservedBits_r16)
                          ? *pscch_config->sl_NumReservedBits_r16 : 0;
  }

  AssertFatal((num_reservedbits>=2) || (num_reservedbits<=4) ,
                      "Num Reserved bits can only be 2or3or4. Resource Pool Configuration Error.\n");
  sci_1a_len += num_reservedbits;
  sci_1a->reserved_bits.nbits = num_reservedbits;
  LOG_D(NR_MAC,"sci 1A - reserved_bits:%d, sci 1a len:%d, sci_1a->reserved_bits.nbits:%d\n",
                                                        num_reservedbits, sci_1a_len, sci_1a->reserved_bits.nbits);


  LOG_D(NR_MAC,"sci 1A Length in bits: %d \n",sci_1a_len);

  return sci_1a_len;
}

/* This function determines the number of sidelink slots in 1024 frames - DFN cycle
* which can be used for determining reserved slots and REsource pool slots according to bitmap.
* Sidelink slots are the uplink and mixed slots with sidelink support except the SSB slots.
*/
uint32_t sl_determine_num_sidelink_slots(uint8_t mod_id, uint16_t *N_SSB_16frames, uint16_t *N_SL_SLOTS_perframe)
{

  NR_UE_MAC_INST_t *mac = get_mac_inst(mod_id);
  sl_nr_ue_mac_params_t *sl_mac = mac->SL_MAC_PARAMS;
  uint32_t N_SSB_1024frames = 0;
  uint32_t N_SL_SLOTS = 0;
  *N_SL_SLOTS_perframe = 0;
  *N_SSB_16frames = 0;

  if (sl_mac->rx_sl_bch.status) {
    sl_ssb_timealloc_t *ssb_timealloc = &sl_mac->rx_sl_bch.ssb_time_alloc;
    *N_SSB_16frames += ssb_timealloc->sl_NumSSB_WithinPeriod;
    LOG_D(MAC, "RX SSB Slots:%d\n", *N_SSB_16frames);
  }

  if (sl_mac->tx_sl_bch.status) {
    sl_ssb_timealloc_t *ssb_timealloc = &sl_mac->tx_sl_bch.ssb_time_alloc;
    *N_SSB_16frames += ssb_timealloc->sl_NumSSB_WithinPeriod;
    LOG_D(MAC, "TX SSB Slots:%d\n", *N_SSB_16frames);
  }

  //Total SSB slots in SFN cycle (1024 frames)
  N_SSB_1024frames = SL_FRAME_NUMBER_CYCLE/SL_NR_SSB_REPETITION_IN_FRAMES * (*N_SSB_16frames);

  sl_nr_phy_config_request_t *sl_cfg = &sl_mac->sl_phy_config.sl_config_req;
  uint8_t sl_scs = sl_cfg->sl_bwp_config.sl_scs;
  uint8_t num_slots_per_frame = 10*(1<<sl_scs);
  uint8_t slot_type = 0;
  for (int i = 0; i < num_slots_per_frame; i++) {
    slot_type = sl_nr_ue_slot_select(sl_cfg, 0, i, TDD);
    if (slot_type == NR_SIDELINK_SLOT) {
      *N_SL_SLOTS_perframe = *N_SL_SLOTS_perframe + 1;
      sl_mac->sl_slot_bitmap |= (1<<i);
    }
  }

  //Determine total number of Valid Sidelink slots which can be used for Respool in a SFN cycle (1024 frames)
  N_SL_SLOTS = (*N_SL_SLOTS_perframe * SL_FRAME_NUMBER_CYCLE) - N_SSB_1024frames;

  LOG_D(MAC, "[UE%d]SL-MAC:SSB slots in 1024 frames:%d, N_SL_SLOTS_perframe:%d, N_SL_SLOTs in 1024 frames:%d, SL SLOT bitmap:%x\n",
                                                                  mod_id,N_SSB_1024frames, *N_SL_SLOTS_perframe,
                                                                  N_SL_SLOTS, sl_mac->sl_slot_bitmap);


  return N_SL_SLOTS;
}


uint8_t count_PSFCH_PRBs_bits(uint8_t* buf, size_t size) {
  uint8_t count = 0;
  uint8_t byte;
  for (size_t i = 0; i < size; i++) {
    byte = buf[i];
    while(byte) {
      count += byte & 1;
      byte >>= 1;
    }
  }
  return count;
}

static void compute_params(int module_idP, psfch_params_t* psfch_params) {
  NR_UE_MAC_INST_t *mac = get_mac_inst(module_idP);
  if (!mac->sl_tx_res_pool->sl_PSFCH_Config_r16 &&
      mac->sl_tx_res_pool->sl_PSFCH_Config_r16->present != NR_SetupRelease_SL_PSFCH_Config_r16_PR_setup)
      return;

  psfch_params->prbs_sets = calloc(1, sizeof(prbs_set_t));
  NR_SL_PSFCH_Config_r16_t *sl_psfch_config = mac->sl_tx_res_pool->sl_PSFCH_Config_r16->choice.setup;
  const int sl_num_muxcs_pair[4] = {1, 2, 3, 6};
  uint8_t sci2_src_id = mac->sci_pdu_rx.source_id;
  uint8_t *rb_buf = sl_psfch_config->sl_PSFCH_RB_Set_r16->buf;
  size_t size = sl_psfch_config->sl_PSFCH_RB_Set_r16->size / sizeof(rb_buf[0]);
  uint8_t m_psfch_prb_set = count_PSFCH_PRBs_bits(rb_buf, size);
  long sl_numsubchannel = *mac->sl_tx_res_pool->sl_NumSubchannel_r16;
  const uint8_t psfch_periods[] = {0,1,2,4};
  long n_psfch_pssch = (sl_psfch_config->sl_PSFCH_Period_r16)
                        ? psfch_periods[*sl_psfch_config->sl_PSFCH_Period_r16] : 0;
  long n_psfch_cs = *sl_psfch_config->sl_NumMuxCS_Pair_r16;

  double m_psfch_subch_slot = m_psfch_prb_set / (sl_numsubchannel * n_psfch_pssch);
  // FIXME: Add second condition from spec. 38213 16.3, current implementation assuming single subchannel
  long n_psfch_type = *sl_psfch_config->sl_PSFCH_CandidateResourceType_r16 ? sl_numsubchannel : 1;
  uint16_t r_psfch_prb_cs = n_psfch_type * m_psfch_subch_slot * sl_num_muxcs_pair[n_psfch_cs];
  uint8_t psfch_rsc_idx = (sci2_src_id + module_idP) / r_psfch_prb_cs;
  LOG_D(NR_MAC, "size %lu, m_psfch_prb_set %d, sl_numsubchannel %ld, n_psfch_pssch %ld, n_psfch_cs %d\n", size, m_psfch_prb_set, sl_numsubchannel, n_psfch_pssch, sl_num_muxcs_pair[n_psfch_cs]);
  LOG_D(NR_MAC, "m_psfch_subch_slot %f, n_psfch_type %ld, r_psfch_prb_cs %d, psfch_rsc_idx %d\n", m_psfch_subch_slot, n_psfch_type, r_psfch_prb_cs, psfch_rsc_idx);
  psfch_params->m0 = table_16_3_1[n_psfch_cs][psfch_rsc_idx];

  // 38213 16.3 Compute PRB allocation
  psfch_params->prbs_sets->start_prb = (uint16_t**)calloc(n_psfch_pssch, sizeof(uint16_t*));
  psfch_params->prbs_sets->end_prb = (uint16_t**)calloc(n_psfch_pssch, sizeof(uint16_t*));
  for (int k=0; k<n_psfch_pssch; k++) {
    psfch_params->prbs_sets->start_prb[k] = (uint16_t*)calloc(sl_numsubchannel, sizeof(uint16_t));
    psfch_params->prbs_sets->end_prb[k] = (uint16_t*)calloc(sl_numsubchannel, sizeof(uint16_t));
  }
  for (int i = 0; i < n_psfch_pssch; i++) {
    for (int j = 0; j < sl_numsubchannel; j++) {
      psfch_params->prbs_sets->start_prb[i][j] = (i + j * n_psfch_pssch) * m_psfch_subch_slot;
      psfch_params->prbs_sets->end_prb[i][j] = (i + 1 + j * n_psfch_pssch) * m_psfch_subch_slot - 1;
    }
  }
}

void configure_psfch_params(int module_idP,
                            NR_UE_MAC_INST_t *mac,
                            sl_nr_rx_indication_t *rx_ind,
                            int pdu_id)
{
  const uint8_t time_gap[] = {2, 3};
  const uint8_t psfch_periods[] = {0,1,2,4};
  NR_SL_PSFCH_Config_r16_t *sl_psfch_config = mac->sl_tx_res_pool->sl_PSFCH_Config_r16->choice.setup;
  long psfch_period = (sl_psfch_config->sl_PSFCH_Period_r16)
                        ? psfch_periods[*sl_psfch_config->sl_PSFCH_Period_r16] : 0;
  uint8_t psfch_min_time_gap = time_gap[*sl_psfch_config->sl_MinTimeGapPSFCH_r16];
  int delta_slots = (rx_ind->slot + psfch_min_time_gap) % psfch_period ? psfch_period - (rx_ind->slot + psfch_min_time_gap) % psfch_period: 0;
  uint16_t sched_slot = rx_ind->slot + psfch_min_time_gap + delta_slots;
  uint16_t sched_frame = rx_ind->sfn;
  int scs = get_softmodem_params()->numerology;
  if (sched_slot >= nr_slots_per_frame[scs]) {
    sched_slot %= nr_slots_per_frame[scs];
    sched_frame = (sched_frame + 1) % 1024;
  }

  uint8_t harq_pid = (rx_ind->rx_indication_body + pdu_id)->rx_slsch_pdu.harq_pid;
  mac->sl_info.list[0] = calloc(1, sizeof(NR_SL_UE_info_t));
  NR_UE_sl_harq_t *harq_proc = &mac->sl_info.list[0]->UE_sched_ctrl.sl_harq_processes[harq_pid];
  harq_proc->feedback_slot = sched_slot;
  harq_proc->feedback_frame = sched_frame;
  harq_proc->is_active = true;

  LOG_D(NR_MAC, "harq pid: %d:%d psfch_period %ld, delta_slots %d, feedback frame:slot %d:%d, frame:slot %d:%d, time_gap %d, harq feedback %d\n",
        harq_pid,
        mac->sl_info.list[0]->UE_sched_ctrl.sl_harq_processes[harq_pid].is_active,
        psfch_period,
        delta_slots,
        harq_proc->feedback_frame,
        harq_proc->feedback_slot,
        rx_ind->sfn,
        rx_ind->slot,
        psfch_min_time_gap,
        mac->sci_pdu_rx.harq_feedback);

  uint8_t ack_nack = (rx_ind->rx_indication_body + pdu_id)->rx_slsch_pdu.ack_nack;
  mac->sl_tx_config_psfch_pdu[harq_pid] = calloc(1, sizeof(sl_nr_tx_config_psfch_pdu_t));
  psfch_params_t *psfch_params = calloc(1, sizeof(psfch_params_t));
  compute_params(module_idP, psfch_params);
  sl_nr_tx_config_psfch_pdu_t *psfch_pdu = mac->sl_tx_config_psfch_pdu[harq_pid];
  psfch_pdu->initial_cyclic_shift = psfch_params->m0;
  if (mac->sci1_pdu.second_stage_sci_format == 2 ||
      mac->sci_pdu_rx.cast_type == 1 ||
      mac->sci_pdu_rx.cast_type == 2) {
    psfch_pdu->mcs = sequence_cyclic_shift_harq_ack_or_ack_or_only_nack[ack_nack];
  } else if (mac->sci1_pdu.second_stage_sci_format == 1 ||
            (mac->sci1_pdu.second_stage_sci_format == 1 && mac->sci_pdu_rx.cast_type == 3)) {
    psfch_pdu->mcs = sequence_cyclic_shift_harq_ack_or_ack_or_only_nack[0];
  }

  const uint8_t values[] = {7, 8, 9, 10, 11, 12, 13, 14};
  NR_SL_BWP_Generic_r16_t *sl_bwp = mac->sl_bwp->sl_BWP_Generic_r16;
  uint8_t sl_num_symbols = *sl_bwp->sl_LengthSymbols_r16 ? values[*sl_bwp->sl_LengthSymbols_r16] : 0;
  // start_symbol_index has been used as lprime check 38.213 16.3
  psfch_pdu->start_symbol_index = *sl_bwp->sl_StartSymbol_r16 + sl_num_symbols - 2;
  LOG_D(NR_PHY, "sl_StartSymbol_r16 %ld, sl_num_symbols: %d, start sym index %d, mcs %d\n", *sl_bwp->sl_StartSymbol_r16, sl_num_symbols, psfch_pdu->start_symbol_index, psfch_pdu->mcs);
  psfch_pdu->hopping_id = *mac->sl_bwp->sl_BWP_PoolConfigCommon_r16->sl_TxPoolSelectedNormal_r16->list.array[0]->sl_ResourcePool_r16->sl_PSFCH_Config_r16->choice.setup->sl_PSFCH_HopID_r16;
  psfch_pdu->prb = psfch_params->prbs_sets->start_prb[rx_ind->slot%psfch_period][0]; // FIXME [0] is based on assumption of number of subchannels = 1; 0 is channel id
  print_prb_set_allocation(psfch_params, psfch_period, 1);
  LOG_D(NR_PHY, "slot %d, slot mode psfch_period %ld, psfch_pdu->prb %d, start_prb %d\n", rx_ind->slot, rx_ind->slot%psfch_period, psfch_pdu->prb, psfch_params->prbs_sets->start_prb[rx_ind->slot%psfch_period][0]);
  int locbw = sl_bwp->sl_BWP_r16->locationAndBandwidth;
  psfch_pdu->sl_bwp_start   = NRRIV2PRBOFFSET(locbw, MAX_BWP_SIZE);
  psfch_pdu->freq_hop_flag  = 0;
  psfch_pdu->group_hop_flag = 0;
  psfch_pdu->second_hop_prb = 0;
  psfch_pdu->bit_len_harq = 1;
  LOG_D(NR_MAC, "Filled psfch pdu\n");
}

void configure_csi_report_params(NR_UE_MAC_INST_t* mac) {
  mac->sl_csi_report = (nr_sl_csi_report_t *) malloc(sizeof(nr_sl_csi_report_t));
  mac->sl_csi_report->CQI = mac->csirs_measurements.cqi;
  mac->sl_csi_report->RI = mac->csirs_measurements.rank_indicator;
  mac->sl_csi_report->R = 0;
}

void nr_ue_process_mac_sl_pdu(int module_idP,
                              sl_nr_rx_indication_t *rx_ind,
                              int pdu_id)
{

  uint8_t *pduP          = (rx_ind->rx_indication_body + pdu_id)->rx_slsch_pdu.pdu;
  int32_t pdu_len        = (int32_t)(rx_ind->rx_indication_body + pdu_id)->rx_slsch_pdu.pdu_length;
  uint8_t done           = 0;
  NR_UE_MAC_INST_t *mac = get_mac_inst(module_idP);
  int frame = rx_ind->sfn;
  int slot = rx_ind->slot;
  if (!pduP){
    return;
  }
  uint8_t psfch_period = 0;
  if (mac->sl_tx_res_pool->sl_PSFCH_Config_r16 && mac->sl_tx_res_pool->sl_PSFCH_Config_r16->choice.setup->sl_PSFCH_Period_r16)
    psfch_period = *mac->sl_tx_res_pool->sl_PSFCH_Config_r16->choice.setup->sl_PSFCH_Period_r16;
  if (psfch_period && mac->sci_pdu_rx.harq_feedback) {
    configure_psfch_params(module_idP, mac, rx_ind, pdu_id);
  }

  if ((rx_ind->rx_indication_body + pdu_id)->rx_slsch_pdu.ack_nack == 0)
    return;

  if (mac->sci_pdu_rx.csi_req) {
    LOG_D(NR_MAC, "%4d.%2d Configuring sl_csi_report parameters\n", frame, slot);
    configure_csi_report_params(mac);
  }

  LOG_D(NR_MAC, "In %s : processing PDU %d (with length %d) of %d total number of PDUs...\n", __FUNCTION__, pdu_id, pdu_len, rx_ind->number_pdus);
  NR_SLSCH_MAC_SUBHEADER_FIXED *sl_sch_subheader = (NR_SLSCH_MAC_SUBHEADER_FIXED *) pduP;
  LOG_D(NR_PHY, "%4d.%2d Rx V %d R %d SRC %d DST %d\n", frame, slot, sl_sch_subheader->V, sl_sch_subheader->R, sl_sch_subheader->SRC, sl_sch_subheader->DST);
  pduP += sizeof(*sl_sch_subheader);
  pdu_len -= sizeof(*sl_sch_subheader);
  while (!done && pdu_len > 0) {
    uint16_t mac_len = 0x0000;
    uint16_t mac_subheader_len = 0x0001; //  default to fixed-length subheader = 1-oct
    uint8_t rx_lcid = ((NR_MAC_SUBHEADER_FIXED *)(pduP))->LCID;
    LOG_D(NR_MAC, "[UE %x] LCID %d, remaining pdu length %d byte(s)\n", mac->src_id, rx_lcid, pdu_len);
    switch (rx_lcid) {
      //  MAC CE
      case SL_SCH_LCID_4_19:
        if (!get_mac_len(pduP, pdu_len, &mac_len, &mac_subheader_len))
          return;
        LOG_D(NR_MAC, "%4d.%2d : SLSCH -> LCID %d %d bytes with subheader %d\n", frame, slot, rx_lcid, mac_len, mac_subheader_len);

        mac_rlc_data_ind(module_idP,
                         mac->src_id,
                         0,
                         frame,
                         ENB_FLAG_NO,
                         MBMS_FLAG_NO,
                         rx_lcid,
                         (char *)(pduP + mac_subheader_len),
                         mac_len,
                         1,
                         NULL);
	      break;
      case SL_SCH_LCID_SL_CSI_REPORT:
        {
          NR_MAC_SUBHEADER_FIXED* sub_pdu_header = (NR_MAC_SUBHEADER_FIXED*) pduP;
          LOG_D(NR_MAC, "LCID %i, R %i\n", sub_pdu_header->LCID, sub_pdu_header->R);
          mac_len = sizeof(*sub_pdu_header);
          nr_sl_csi_report_t* nr_sl_csi_report = (nr_sl_csi_report_t *) (pduP + mac_len);
          LOG_D(NR_MAC, "%4d.%2d: CQI: %i RI: %i\n", frame, slot, nr_sl_csi_report->CQI, nr_sl_csi_report->RI);
          break;
        }
      case SL_SCH_LCID_SL_PADDING:
	      LOG_D(NR_MAC, "Received padding\n");
	      done = 1;
	      break;
      case SL_SCH_LCID_SCCH_PC5_NOT_PROT:
      case SL_SCH_LCID_SCCH_PC5_DSMC:
      case SL_SCH_LCID_SCCH_PC5_PROT:
      case SL_SCH_LCID_SCCH_PC5_RRC:
      case SL_SCH_LCID_20_55:
      case SL_SCH_LCID_SCCH_RRC_SL_RLC0:
      case SL_SCH_LCID_SCCH_RRC_SL_RLC1:
      case SL_SCH_LCID_SCCH_SL_DISCOVERY:
      case SL_SCH_LCID_SL_INTER_UE_COORD_REQ:
      case SL_SCH_LCID_SL_INTER_UE_COORD_INFO:
      case SL_SCH_LCID_SL_DRX_CMD:
	      LOG_W(NR_MAC,"Received unsupported SL LCID %d\n",rx_lcid);
	      return;
	      break;
    }
    pduP += ( mac_subheader_len + mac_len );
    pdu_len -= ( mac_subheader_len + mac_len );
    LOG_D(NR_MAC, "mac_subhead_len + mac_len = %d\n", mac_subheader_len + mac_len);
    LOG_D(NR_MAC, "%4d.%2d : SLSCH -> LCID %d remaining pdu length %d byte(s)\n", frame, slot, rx_lcid, pdu_len);
    if (pdu_len < 0)
      LOG_E(NR_MAC, "[UE %d][%d.%d] nr_ue_process_mac_pdu_sl, residual mac pdu length %d < 0!\n", module_idP, frame, slot, pdu_len);
  }
}

/**
 * \file
 *
 * \brief SAM D2x TCC Driver Quick Start with DMA
 *
 * Copyright (C) 2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \page asfdoc_sam0_tcc_dma_use_case Quick Start Guide for Using DMA with TCC
 *
 * The supported device list:
 *    - SAMD21
 *
 * In this use case, the TCC will be used to generate a PWM signal. Here
 * the pulse width varies in following values through DMA transfer: one quater
 * of the period, half of the period and three quaters of the period.
 * When connect PWM output to LED it makes the LED light. To see the waveform,
 * you may need an ossiliscope.
 * The output signal is also fed back to another TCC channel by event system,
 * the event stamps are captured and transfer to values buffer by DMA.
 *
 * The PWM output is set up as follows:
 * <table>
 *  <tr><th> board        </td><th> pin  </td><th> connect to </td></tr>
 *  <tr><td> SAMD21 Xpro  </td><td> PB30 </td><td> LED0       </td></tr>
 * </table>
 *
 * The TCC module will be setup as follows:
 * - GCLK generator 0 (GCLK main) clock source
 * - No dithering on the counter or compare
 * - No prescaler
 * - Single Slope PWM wave generation
 * - GCLK reload action
 * - Don't run in standby
 * - No fault or waveform extensions
 * - No inversion of waveform output
 * - No capture enabled
 * - Count upward
 * - Don't perform one-shot operations
 * - No event input enabled
 * - No event action
 * - No event generation enabled
 * - Counter starts on 0
 * - Counter top set to 0x1000
 * - Channel 0 is set to compare and match value 0x1000*3/4
 * - Channel 1 is set to capture input event
 * - Channel 0 compare generates event
 * - Channel 1 perform capture when there is channel event
 *
 * The event resource of EVSYS module will be setup as follows:
 * - TCC match capture channel 0 is selected as event generator
 * - Event generation is synchronous, with rising edge detected
 * - TCC match capture channel 1 is the event resource user
 *
 * The DMA resource of DMAC module will be setup as follows:
 * - Two DMA resources is used for compare and capture
 * - Both DMA resources use peripheral trigger
 * - Both DMA resources perform beat transfer on trigger
 * - Both DMA resources transfer 16-bit in a beat
 * - Both DMA will transfer 3 beats and then repeat again in same buffer
 * - On DMA resource for compare
 *   - TCC overflow will trigger DMA beat
 *   - The source address is increment
 *   - The destination address is fixed to TCC channel 0 match capture register
 *   - The increment step size is defined by destination - TCC
 * - On DMA resource for capture
 *   - TCC capture on channel 1 will trigger DMA beat
 *   - The source address is fixed to TCC channel 1 match capture register
 *   - The destination address is increment
 *   - The increment step size is defined by source - TCC
 *
 * \section asfdoc_sam0_tcc_dma_use_case_setup Quick Start
 *
 * \subsection asfdoc_sam0_tcc_dma_use_case_prereq Prerequisites
 * There are no prerequisites for this use case.
 *
 * \subsection asfdoc_sam0_tcc_dma_use_case_setup_code Code
 *
 * Add to the main application source file, before any functions:
 * \snippet conf_quick_start_dma.h definition_pwm
 * \snippet conf_quick_start_dma.h definition_feedback
 * \snippet conf_quick_start_dma.h definition_dma_compare_trigger
 * \snippet conf_quick_start_dma.h definition_dma_capture_trigger
 *
 * Add to the main application source file, outside of any functions:
 * \snippet qs_tcc_dma.c module_inst
 * \snippet qs_tcc_dma.c capture_variables
 * \snippet qs_tcc_dma.c compare_variables
 *
 * Copy-paste the following setup code to your user application:
 * \snippet qs_tcc_dma.c config_event_for_capture
 * \snippet qs_tcc_dma.c config_dma_for_capture
 * \snippet qs_tcc_dma.c config_dma_for_wave
 * \snippet qs_tcc_dma.c setup
 *
 * Add to user application initialization (typically the start of \c main()):
 * \snippet qs_tcc_dma.c setup_init
 *
 * \subsection asfdoc_sam0_tcc_dma_use_case_setup_flow Workflow
 * \subsubsection asfdoc_sam0_tcc_dma_use_case_setup_flow_tcc Configure the TCC
 * -# Create a module software instance structure for the TCC module to store
 *    the TCC driver state while it is in use.
 *    \snippet qs_tcc_dma.c module_inst
 *    \note This should never go out of scope as long as the module is in use.
 *          In most cases, this should be global.
 * -# Create a TCC module configuration struct, which can be filled out to
 *     adjust the configuration of a physical TCC peripheral.
 *     \snippet qs_tcc_dma.c setup_config
 * -# Initialize the TCC configuration struct with the module's default values.
 *     \snippet qs_tcc_dma.c setup_config_defaults
 *     \note This should always be performed before using the configuration
 *           struct to ensure that all values are initialized to known default
 *           settings.
 * -# Alter the TCC settings to configure the counter width, wave generation
 *     mode and the compare channel 0 value.
 *     \snippet qs_tcc_dma.c setup_change_config
 * -# Alter the TCC settings to configure the PWM output on a physical device
 *     pin.
 *     \snippet qs_tcc_dma.c setup_change_config_pwm
 * -# Configure the TCC module with the desired settings.
 *     \snippet qs_tcc_dma.c setup_set_config
 * -# Configure and enable the desired events for TCC module.
 *     \snippet qs_tcc_dma.c setup_events
 * \subsubsection asfdoc_sam0_tcc_dma_use_case_setup_flow_event Configure the Event System
 * Configure the EVSYS module to wire channel 0 event to channel 1.
 * -# Create an event resource instance.
 *     \snippet qs_tcc_dma.c capture_event_resource
 *     \note This should never go out of scope as long as the resource is in
 *     use. In most cases, this should be global.
 *
 * -# Create an event resource configuration struct.
 *      \snippet qs_tcc_dma.c event_setup_1
 * -# Initialize the event resource configuration struct with default values.
 *      \snippet qs_tcc_dma.c event_setup_2
 *      \note This should always be performed before using the configuration
 *            struct to ensure that all values are initialized to known default
 *            settings.
 * -# Adjust the event resource configuration to desired values.
 *      \snippet qs_tcc_dma.c event_setup_3
 * -# Allocate and configure the resource using the configuration structure.
 *      \snippet qs_tcc_dma.c event_setup_4
 * -# Attach an user to the resource
 *      \snippet qs_tcc_dma.c event_setup_5
 * \subsubsection asfdoc_sam0_tcc_dma_use_case_setup_flow_dma_capture Configure the DMA for Capture TCC Channel 1
 * Configure the DMAC module to obtain captured value from TCC channel 1.
 * -# Create a DMA resource instance.
 *     \snippet qs_tcc_dma.c capture_dma_resource
 *     \note This should never go out of scope as long as the resource is in
 *             use. In most cases, this should be global.
 * -# Create a DMA resource configuration struct.
 *       \snippet qs_tcc_dma.c dma_setup_1
 * -# Initialize the DMA resource configuration struct with default values.
 *       \snippet qs_tcc_dma.c dma_setup_2
 *       \note This should always be performed before using the configuration
 *             struct to ensure that all values are initialized to known default
 *             settings.
 * -# Adjust the DMA resource configurations.
 *       \snippet qs_tcc_dma.c dma_setup_3
 * -# Allocate a DMA resource with the configurations.
 *       \snippet qs_tcc_dma.c dma_setup_4
 * -# Prepare DMA transfer descriptor
 *  -# Create a DMA transfer descriptor.
 *       \snippet qs_tcc_dma.c capture_dma_descriptor
 *       \note When multiple descriptors are linked. The linked item should
 *             never go out of scope before it's loaded (to DMA Write-Back
 *             memory section). In most cases, if more than one descriptors are
 *             used, they should be global except the very first one.
 *  -# Create a DMA transfer descriptor struct.
 *  -# Create a DMA transfer descriptor configuration structure, which can be
 *       filled out to adjust the configuration of a single DMA transfer.
 *       \snippet qs_tcc_dma.c dma_setup_5
 *  -# Initialize the DMA transfer descriptor configuration struct with
 *       default values.
 *       \snippet qs_tcc_dma.c dma_setup_6
 *       \note This should always be performed before using the configuration
 *             struct to ensure that all values are initialized to known default
 *             settings.
 *  -# Adjust the DMA transfer descriptor configurations.
 *       \snippet qs_tcc_dma.c dma_setup_7
 *  -# Create the DMA transfer descriptor with configuration.
 *       \snippet qs_tcc_dma.c dma_setup_8
 *  -# Adjust the DMA transfer descriptor if multiple DMA transfer will be
 *        performed.
 *       \snippet qs_tcc_dma.c dma_setup_9
 * -# Start DMA transfer job with prepared descriptor
 *  -# Add the DMA transfer descriptor to the allocated DMA resource.
 *       \snippet qs_tcc_dma.c dma_setup_10
 *  -# Start the DMA transfer job with the allocated DMA resource and
 *       transfer descriptor.
 *       \snippet qs_tcc_dma.c dma_setup_11
 * \subsubsection asfdoc_sam0_tcc_dma_use_case_setup_flow_dma_compare Configure the DMA for Compare TCC Channel 0
 * Configure the DMAC module to update TCC channel 0 compare value. The flow is similar to last DMA configure step for capture.
 * -# Allocate and configure the DMA resource
 *     \snippet qs_tcc_dma.c compare_dma_resource
 *     \snippet qs_tcc_dma.c config_dma_resource_for_wave
 * -# Prepare DMA transfer descriptor
 *     \snippet qs_tcc_dma.c compare_dma_descriptor
 *     \snippet qs_tcc_dma.c config_dma_descriptor_for_wave
 * -# Start DMA transfer job with prepared descriptor
 *     \snippet qs_tcc_dma.c config_dma_job_for_wave
 * -# Enable the TCC module to start the timer and begin PWM signal generation.
 *     \snippet qs_tcc_dma.c setup_enable
 *
 * \section asfdoc_sam0_tcc_dma_use_case_main Use Case
 *
 * \subsection asfdoc_sam0_tcc_dma_use_case_main_code Code
 * Copy-paste the following code to your user application:
 * \snippet qs_tcc_dma.c main
 *
 * \subsection asfdoc_sam0_tcc_dma_use_case_main_flow Workflow
 * -# Enter an infinite loop while the PWM wave is generated via the TCC module.
 *  \snippet qs_tcc_dma.c main_loop
 */

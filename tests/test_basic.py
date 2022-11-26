import logging

import pytest

from pi_nbiot import waveshare


def test_basic():
    hat = waveshare.WaveshareNbiotHat()
    hat.initialize()
    hat.pdp_context_define(0, 'ciot')
    hat.pdp_context_configure(0)
    hat.pdp_context_activate(0)
    assert hat.active_pdp_context == 0
    pdp_context: waveshare.PdpContext = hat.pdp_contexts[0]
    assert pdp_context.is_ip

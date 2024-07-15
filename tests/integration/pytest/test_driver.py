# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0

import logging
from packaging import version
import re

from twister_harness import Shell

logger = logging.getLogger(__name__)

def catch(func, handle=lambda e : e, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except Exception as e:
        return False

def test_shell_ping(shell: Shell):
    logger.info('send "ping" command')
    lines = shell.exec_command('test ping')
    assert 'pong' in lines, 'expected response not found'
    logger.info('response is valid')


def test_smd_ping(shell: Shell):
    logger.info('send "ping" command to SMD')
    lines = shell.exec_command('test smd')
    assert '+OK' in lines, 'expected response not found'
    logger.info('response is valid')
    
def test_shell_version(shell: Shell):
    logger.info('send "test version" command')
    lines = shell.exec_command('test version')
    expected_pattern = r'\+FW=[0-9a-f]{7}__0x[0-9a-f]+,\w{3}\s+\d{1,2}\s+\d{4}_\d{2}:\d{2}:\d{2}'
    assert any(re.search(expected_pattern, line) for line in lines), 'expected response not found'
    logger.info('response is valid')
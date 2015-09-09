# Copyright 2014 Chris Matteri
# Released under the MIT License (http://opensource.org/licenses/mit-license.php)

from setup import ExtensionInstaller

def loader():
    return CCWXRXVP2Installer()

class CCWXRXVP2Installer(ExtensionInstaller):
    def __init__(self):
        super(CCWXRXVP2Installer, self).__init__(
            version="1.0",
            name='ccwxrxvp2',
            description='Driver for the Davis Vantage Pro 2 used with the '
                        'CC1101 Weather Receiver.',
            author="Chris Matteri",
            author_email="chrismatteri@gmail.com",
            config={
                'Station': {
                    'station_type': 'CCWXRXVP2'},
                'CCWXRXVP2': {
                    'poll_interval': '2.5625',
                    'driver': 'user.ccwxrxvp2'}},
            files=[('bin/user', ['bin/user/ccwxrxvp2.py'])]
            )

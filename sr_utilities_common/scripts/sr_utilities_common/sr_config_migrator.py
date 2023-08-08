#!/usr/bin/python3.8
import subprocess
import os
import tempfile
import ruamel.yaml
import argparse



class SrConfigMigratorCloneRepos:
    _SR_CONFIG_LOCATION = '/tmp/sr-config'
    _SR_HAND_CONFIG_LOCATION = '/tmp/sr_hand_config'

    def __init__(self, sr_config_branch) -> None:
        self._git_repo_locations = [self._SR_CONFIG_LOCATION, self._SR_HAND_CONFIG_LOCATION]
        self._sr_config_name = 'sr-config'
        self._sr_hand_config_name = 'sr_hand_config'
        self._temp_folder = tempfile.mkdtemp() # '/tmp/tmpm1ttqpap'  #
        self._clone_repos()
        self._checkout_repos(sr_config_branch)

    
    def _check_branch_exists(self, sr_config_path, branch_name):
        does_branch_exist = int(subprocess.run(["bash", "-c", f"cd {sr_config_path} && git branch -a | grep {branch_name} | wc -l"],
                                           capture_output=True,
                                           text=True).stdout.replace('\n', ''))
        if does_branch_exist == 0:
            print(f"Cannot find sr-config branch: {branch_name}, exiting...")
            exit(0)
    
    def _checkout_repos(self, sr_config_branch):
        self._checkout_branch(self._sr_config_name, sr_config_branch)
        self._checkout_branch(self._sr_hand_config_name, sr_config_branch, new_branch=True)
    
    def _checkout_branch(self, repo_name, branch_name, new_branch=False):
        sr_config_path = os.path.join(self._temp_folder, repo_name)
        if new_branch:
            checkout_command = f"git checkout -b adding_{branch_name}"
        else:
            self._check_branch_exists(sr_config_path, branch_name)
            checkout_command = f"git checkout {branch_name}"
        subprocess.run(["bash", "-c", f"cd {sr_config_path} && {checkout_command}"])

    def _clone_repos(self):
        self._clone_repo(self._sr_config_name)
        # import shutil
        # shutil.rmtree(os.path.join(self._temp_folder, self._sr_hand_config_name))
        self._clone_repo(self._sr_hand_config_name)

    def _get_temp_folder_repo_path(self, repo_name):
        return os.path.join(self._temp_folder, repo_name)

    def _clone_repo(self, repo_name):
        github_repo_url = f'http://github.com/shadow-robot/{repo_name}'
        local_repo_location = self._get_temp_folder_repo_path(repo_name)
        subprocess.run(['git', 'clone', github_repo_url, local_repo_location])
    
    def get_path_dict(self):
        path_dict = {}
        path_dict[self._sr_config_name] = self._get_temp_folder_repo_path(self._sr_config_name)
        path_dict[self._sr_hand_config_name] = self._get_temp_folder_repo_path(self._sr_hand_config_name)
        path_dict['temp_folder'] = self._temp_folder
        return path_dict


class SrConfigMigrator:
    _ALLOWED_SIDES = ['left', 'right']
    _ALLOWED_TACTILES = ['bt_sp', 'bt_2p', 'pst', 'mst']
    _ALLOWED_VERSIONS = ['E3M5', 'G1M5', 'E4']
    def __init__(self, side, hand_version, tactile_type, hand_serial, sr_config_branch):
        self._validate_inputs(side, hand_version, tactile_type)
        self._prefix = f"{side[0]}h"
        self._hand_version = hand_version
        self._tactile_type = tactile_type
        self._side = side
        self._hand_serial = str(hand_serial)
        self._sr_config_branch = sr_config_branch
        self._new_mapping_path = None
        self._temp_folder = None
        self._ind = None
        self._bsi = None
        self.run()
    
    def run(self):
        repo_cloner = SrConfigMigratorCloneRepos(self._sr_config_branch)
        self._temp_path_dict = repo_cloner.get_path_dict()
        self._temp_folder = self._temp_path_dict['temp_folder']
        print(f"\n\nCreated temp folder to work in: {self._temp_path_dict['temp_folder']}\n\n")

        sr_hand_config_dir = os.path.join(self._temp_path_dict['temp_folder'], 'sr_hand_config', 'sr_hand_config')
        sr_hand_config_serial_path = os.path.join(sr_hand_config_dir, self._hand_serial)
        self._copy_template(sr_hand_config_dir, sr_hand_config_serial_path)
        
        self._update_general_info(sr_hand_config_serial_path)
        self._update_calibrations_folder()
        self._update_controls_folder()
        self._update_rates_folder()
        print(f"\n\nAll done. Please run the following to push the newly created branch to github:\n")
        print(f"cd {self._temp_path_dict['sr_hand_config']} ")#&& git push origin/adding_{self._sr_config_branch}\n\n")
        print(f"\n\ncat {self._temp_path_dict['sr_hand_config']}/sr_hand_config/{self._hand_serial}/general_info.yaml\n")
        '''
        general_info_out = subprocess.run(["bash", "-c", f"cat {self._temp_path_dict['sr_hand_config']}/sr_hand_config/{self._hand_serial}/general_info.yaml"],
                                      capture_output = True,
                                      text = True).stdout
        print(general_info_out)

        pressure_hand_config = subprocess.run(["bash", "-c", f"cat {self._temp_path_dict['sr_hand_config']}/sr_hand_config/{self._hand_serial}/calibrations/pressure_calibration.yaml"],
                                capture_output = True,
                                text = True).stdout
        
        pressure_config = subprocess.run(["bash", "-c", f"cat {self._temp_path_dict['sr-config']}/sr_ethercat_hand_config/calibrations/lh/pressure_calibration.yaml"],
                                capture_output = True,
                                text = True).stdout
        print(f"\npressure_hand_config:\n{pressure_hand_config}\n\npressure_config:\n{pressure_config}\n\n")
        '''

    def _update_rates_folder(self):
        sr_hand_config_rates_folder = self._get_sr_hand_config_rates_folder()
        sr_config_rates_folder = os.path.join(self._temp_path_dict['sr-config'],
                                             'sr_ethercat_hand_config',
                                             'rates')
        self._update_all_files_in_folder(sr_config_folder=sr_config_rates_folder,
                                         sr_hand_config_folder=sr_hand_config_rates_folder)
    
    def _update_controls_folder(self):
        sr_hand_config_controls_folder = self._get_sr_hand_config_controls_folder()
        self._update_controls_host(sr_hand_config_controls_folder)
        self._update_controls_motors(sr_hand_config_controls_folder)
        self._update_controls_tactiles(sr_hand_config_controls_folder)
    
    def _update_controls_tactiles(self, sr_hand_config_controls_folder):
        sr_hand_config_tactiles_file = os.path.join(sr_hand_config_controls_folder,
                                                    'tactiles',
                                                    'sr_tactile_sensor_controller.yaml')
        sr_config_tactiles_folder = os.path.join(self._temp_path_dict['sr-config'],
                        'sr_ethercat_hand_config',
                        'controls',
                        'tactiles',
                        f'{self._prefix}')
        if not os.path.exists(sr_config_tactiles_folder):
            sr_config_tactiles_folder_other = os.path.join(self._temp_path_dict['sr-config'],
                        'sr_ethercat_hand_config',
                        'controls',
                        'tactiles')
            if not os.path.exists(sr_config_tactiles_folder):
                print(f"Could not find tactile info in sr-config at")
                print(f"  {sr_config_tactiles_folder.replace(self._temp_folder, '')}")
                print("or")
                print(f"  {sr_config_tactiles_folder_other.replace(self._temp_folder, '')}")
                print(f"Not updating {sr_hand_config_tactiles_file.replace(self._temp_folder, '')}!")
                return
            sr_config_tactiles_folder = sr_config_tactiles_folder_other
        sr_config_tactiles_file = os.path.join(sr_config_tactiles_folder, 'sr_tactile_sensor_controller.yaml')
        self._update_file(sr_config_fname=sr_config_tactiles_file,
                          sr_hand_config_fname=sr_hand_config_tactiles_file,
                          key='all')
        
    def _update_controls_motors(self, sr_hand_config_controls_folder):
        sr_hand_config_motors_folder = os.path.join(sr_hand_config_controls_folder, 'motors')
        sr_config_motors_folder = os.path.join(self._temp_path_dict['sr-config'],
                        'sr_ethercat_hand_config',
                        'controls',
                        'motors',
                        f'{self._prefix}')

        sr_config_motors_file = os.path.join(sr_config_motors_folder, 'motor_board_effort_controllers.yaml')
        sr_hand_config_motors_file = os.path.join(sr_hand_config_motors_folder, 'motor_board_effort_controllers.yaml')
        self._update_file(sr_config_fname=sr_config_motors_file,
                          sr_hand_config_fname=sr_hand_config_motors_file,
                          key='all')
        
    def _update_controls_host(self, sr_hand_config_controls_folder):
        sr_hand_config_host_folder = os.path.join(sr_hand_config_controls_folder, 'host')
        sr_config_host_folder = os.path.join(self._temp_path_dict['sr-config'],
                                'sr_ethercat_hand_config',
                                'controls',
                                'host',
                                f'{self._prefix}')
        self._update_controls_host_common(sr_hand_config_host_folder)
        self._update_controls_host_pwm(sr_hand_config_host_folder, sr_config_host_folder)
        self._update_controls_host_torque(sr_hand_config_host_folder, sr_config_host_folder)

    def _update_controls_host_common(self, host_folder):
        common_folder = os.path.join(host_folder, 'common')
        friction_compensation_sr_hand_config_fname = os.path.join(common_folder, 'friction_compensation.yaml')
        friction_compensation_sr_config_fname = os.path.join(self._temp_path_dict['sr-config'],
                                                                            'sr_ethercat_hand_config',
                                                                            'controls',
                                                                            'friction_compensation.yaml')
        self._update_file(sr_config_fname=friction_compensation_sr_config_fname,
                          sr_hand_config_fname=friction_compensation_sr_hand_config_fname,
                          key='sr_friction_map')
    
    def _update_controls_host_pwm(self, sr_hand_config_host_folder, sr_config_host_folder):
        sr_hand_config_pwm_folder = os.path.join(sr_hand_config_host_folder, 'pwm')
        self._update_all_files_in_folder(sr_config_folder=sr_config_host_folder,
                                         sr_hand_config_folder=sr_hand_config_pwm_folder)

    
    def _update_controls_host_torque(self, sr_hand_config_host_folder, sr_config_host_folder):
        sr_hand_config_torque_folder = os.path.join(sr_hand_config_host_folder, 'torque')
        self._update_all_files_in_folder(sr_config_folder=sr_config_host_folder,
                                         sr_hand_config_folder=sr_hand_config_torque_folder)
        
    def _get_sr_hand_config_controls_folder(self):
        return os.path.join(self._temp_path_dict['sr_hand_config'],
                            'sr_hand_config',
                            self._hand_serial,
                            'controls')
    
    def _get_sr_hand_config_rates_folder(self):
        return os.path.join(self._temp_path_dict['sr_hand_config'],
                            'sr_hand_config',
                            self._hand_serial,
                            'rates')

    def _update_all_files_in_folder(self, sr_config_folder, sr_hand_config_folder):
        for sr_hand_config_file in os.listdir(sr_hand_config_folder):
            sr_hand_config_fname = os.path.join(sr_hand_config_folder, sr_hand_config_file)
            sr_config_fname = os.path.join(sr_config_folder, sr_hand_config_file)
            self._update_file(sr_config_fname=sr_config_fname,
                              sr_hand_config_fname=sr_hand_config_fname,
                              key='all')
    
    def _recursive_key_update(self, config_data, hand_config_data):
        for key in hand_config_data.keys():
            if key in hand_config_data.keys():
                if isinstance(config_data[key], dict):
                    self._recursive_key_update(config_data[key], hand_config_data[key])
                else:
                    hand_config_data[key] = config_data[key]
            else:
                hand_config_data[key] = config_data[key]
    
    def _recursive_update(self, config_data, hand_config_data):
        if isinstance(hand_config_data, ruamel.yaml.comments.CommentedMap):
            for key in hand_config_data.keys():
                if isinstance(hand_config_data[key], ruamel.yaml.comments.CommentedSeq) or \
                isinstance(hand_config_data[key], ruamel.yaml.comments.CommentedMap):
                    self._recursive_update(config_data[key], hand_config_data[key])
                else:
                    hand_config_data[key] = config_data[key]
        elif isinstance(hand_config_data, ruamel.yaml.comments.CommentedSeq):
            for seq in hand_config_data:
                if isinstance(seq, ruamel.yaml.comments.CommentedSeq) or \
                isinstance(seq, ruamel.yaml.comments.CommentedMap):
                    self._recursive_update(config_data[key], hand_config_data[key])
                else:
                    hand_config_data[key] = config_data[key]
        return hand_config_data
            # ruamel.yaml.comments.CommentedSeq
            # ruamel.yaml.comments.CommentedMap
    
    def recursive_update_2(self, config_data, hand_config_data):
        if isinstance(hand_config_data, ruamel.yaml.comments.CommentedMap):
            for key in hand_config_data.keys():
                if isinstance(hand_config_data[key], ruamel.yaml.comments.CommentedSeq) or \
                isinstance(hand_config_data[key], ruamel.yaml.comments.CommentedMap):
                    self._recursive_update_2()

    def _update_file(self, sr_config_fname, sr_hand_config_fname, key):
        sr_config_data = self._load_yaml(sr_config_fname)
        sr_hand_config_data = self._load_yaml(sr_hand_config_fname)
        sr_hand_config_data.update(sr_config_data)
        sr_hand_config_data.copy_attributes(sr_config_data)
        self._update_yaml(sr_hand_config_fname, sr_hand_config_data)
        return
        if 'calibration' in sr_config_fname and 'pressure' not in sr_config_fname:
            a=1
        if key != 'all':
            sr_hand_config_data[key] = sr_config_data[key]
        else:
            # self._locator_list = []
            # self._recursive_key_update(sr_config_data, sr_hand_config_data)
            for found_key in sr_config_data.keys():
                sr_hand_config_data[found_key] = sr_config_data[found_key]
        sr_hand_config_data.copy_attributes(sr_config_data)
        self._update_yaml(sr_hand_config_fname, sr_hand_config_data)
    
    def _update_calibrations_folder(self):
        # self._update_pressure_calibration_file_old_yaml()
        # self._update_calibration_file()
        sr_config_calibrations_folder = os.path.join(self._temp_path_dict['sr-config'],
                                                     'sr_ethercat_hand_config',
                                                     'calibrations',
                                                     f'{self._prefix}')
        sr_hand_config_calibrations_folder = os.path.join(self._temp_path_dict['sr_hand_config'],
                                                        'sr_hand_config',
                                                        self._hand_serial,
                                                        'calibrations')
        self._update_all_files_in_folder(sr_config_folder=sr_config_calibrations_folder,
                                         sr_hand_config_folder=sr_hand_config_calibrations_folder)
        
    
    def _update_pressure_calibration_file_old_yaml(self):
        sr_config_pressure_calibration_fname = os.path.join(self._temp_path_dict['sr-config'],
                                                     'sr_ethercat_hand_config',
                                                     'calibrations',
                                                     f'{self._prefix}',
                                                     "pressure_calibration.yaml")
        sr_hand_config_pressure_calibration_fname = os.path.join(self._temp_path_dict['sr_hand_config'],
                                                        'sr_hand_config',
                                                        self._hand_serial,
                                                        'calibrations',
                                                        'pressure_calibration.yaml')
        sr_config_stream = open(sr_config_pressure_calibration_fname)
        # Annoyingly, the pressure_calibration.yaml file breaks the yaml load/save used elsewhere in this script
        # So we use a different approach here
        pressure_calibration_sr_config = pressure_yaml.load(sr_config_stream)
        yaml = YAML(typ='rt')
        yaml.preserve_quotes = True
        stream = open(sr_hand_config_pressure_calibration_fname)
        pressure_calibration_sr_hand_config = pressure_yaml.load(stream)

        for x in pressure_calibration_sr_config['sr_pressure_calibrations']:
            print(x)
        print('\n\n\n')
        for x in pressure_calibration_sr_hand_config['sr_pressure_calibrations']:
            print(x)
        pressure_calibration_sr_hand_config['sr_pressure_calibrations'] = pressure_calibration_sr_config['sr_pressure_calibrations']
        self._update_yaml(sr_hand_config_pressure_calibration_fname, pressure_calibration_sr_hand_config)
    
    def _update_calibration_file(self):
        sr_config_calibration_fname = os.path.join(self._temp_path_dict['sr-config'],
                                                     'sr_ethercat_hand_config',
                                                     'calibrations',
                                                     f'{self._prefix}',
                                                     "calibration.yaml")
        calibration_sr_config = self._load_yaml(sr_config_calibration_fname)
        sr_hand_config_calibration_fname = os.path.join(self._temp_path_dict['sr_hand_config'],
                                                        'sr_hand_config',
                                                        self._hand_serial,
                                                        'calibrations',
                                                        'calibration.yaml')
        calibration_sr_hand_config = self._load_yaml(sr_hand_config_calibration_fname)
        calibration_sr_hand_config['sr_calibrations'] = calibration_sr_config['sr_calibrations']
        if len(calibration_sr_config.keys()) > 1:
            calibration_sr_hand_config['sr_calibrations_coupled'] = calibration_sr_config['sr_calibrations_coupled']
        
        self._update_yaml(sr_hand_config_calibration_fname, calibration_sr_hand_config)
        
    
    def _get_calibrations_folder(self):
        calibrations_folder = os.path.join(self._temp_path_dict['sr-config'], 'sr_ethercat_hand_config', 'calibrations')
        hand_subfolder = os.path.join(calibrations_folder, f'{self._prefix}')
        if os.path.exists(hand_subfolder):
            return hand_subfolder
        return calibrations_folder
    
    def _get_new_mapping_path(self):
        sr_config_hand_launch_file = os.path.join(self._temp_path_dict['sr-config'],
                                    'sr_ethercat_hand_config',
                                    'launch',
                                    f'sr_{self._side[0]}hand.launch')
        new_mapping_path = subprocess.run(["bash", "-c", f"cat {sr_config_hand_launch_file} | grep mapping_path"\
                                       " | sed -r 's/.*sr_edc_launch\)//g'"\
                                       " | sed -r 's/\".*//g'"],
                                      capture_output = True,
                                      text = True).stdout.replace('\n', '')
        if new_mapping_path == '':
            print(f"Cannot infer mapping path from sr-config branch")
            new_mapping_path = input("Please enter a relative mapping path, e.g: mappings/default_mappings/rh_E_v4.yaml\n\n")
        return new_mapping_path

    def _copy_template(self, sr_hand_config_dir, sr_hand_config_serial_path):
        if os.path.exists(sr_hand_config_serial_path):
            print(f"Folder {sr_hand_config_serial_path.replace(self._temp_folder, '')} already exists. Has this hand already been ported? Exiting...")
            exit(0)
        os.mkdir(sr_hand_config_serial_path)
        template_folder_name = f"Template_{self._hand_version[0].upper()}{self._prefix[0].upper()}"
        template_to_copy = os.path.join(sr_hand_config_dir, template_folder_name)
        print(f"Copying template from {template_folder_name} to "\
              f"{sr_hand_config_serial_path.replace(self._temp_path_dict['sr_hand_config'], '')}")
        subprocess.run(['cp', '-rT', template_to_copy , sr_hand_config_serial_path])
    
    def _load_yaml(self, fname):
        yaml = ruamel.yaml.YAML(typ='rt')
        yaml.preserve_quotes = True
        stream = open(fname)
        return yaml.load(stream)

    def _update_yaml(self, fname, data):
        print(f"  Updating {fname.replace(self._temp_path_dict['sr_hand_config'], '')}...")
        output_file = open(fname, 'w')
        try:
            ruamel.yaml.round_trip_dump(data, output_file)
            return
        except IndexError:
            pass

        try:
            ruamel.yaml.round_trip_dump(dict(data), output_file)
            return
        except IndexError:
            print(f"Error, failed to write: {fname}")
            print("This is probably a problem.")
            user_input = input("\nPress y to continue, or anything else to exit\n")
            if user_input.lower() != 'y':
                exit(0)


    def _update_general_info(self, sr_hand_config_serial_path):
        new_mapping_path = self._get_new_mapping_path()
        fname = os.path.join(sr_hand_config_serial_path, "general_info.yaml")
        general_info_data = self._load_yaml(fname)
        general_info_data = self._check_update_general_info_tactiles(general_info_data)
        general_info_data = self._check_update_version(general_info_data)
        general_info_data = self._check_update_mapping_path(general_info_data, new_mapping_path)

        self._update_yaml(fname, general_info_data)
        # with open(fname, 'w') as yaml_file:
        #     yaml.dump(general_info_data, yaml_file)

    def _check_update_general_info_tactiles(self, general_info_data):
        template_tactile_type = general_info_data['sensors']['tip']['th']
        if template_tactile_type != self._tactile_type:
            for distal in general_info_data['sensors']['tip']:
                general_info_data['sensors']['tip'][distal] = self._tactile_type
        return general_info_data

    def _check_update_version(self, general_info_data):
        template_version = general_info_data['version']
        if template_version != self._hand_version:
            general_info_data['version'] = self._hand_version
        return general_info_data

    def _check_update_mapping_path(self, general_info_data, new_mapping_path):
        template_mapping_path = general_info_data['mapping_path']['relative_path']
        if template_mapping_path != new_mapping_path:
            general_info_data['mapping_path']['relative_path'] = new_mapping_path
        return general_info_data

    def _validate_inputs(self, side, hand_version, tactile_type):
        if not side in self._ALLOWED_SIDES:
            print(f"side '{side}' not allowed. Allowed sides are: '{self._ALLOWED_SIDES}'")
            exit(0)
        if not hand_version in self._ALLOWED_VERSIONS:
            print(f"hand_version '{hand_version}' not allowed. Allowed hand versions are: '{self._ALLOWED_VERSIONS}'")
            exit(0)
        if not tactile_type in self._ALLOWED_TACTILES:
            print(f"tactile_type '{tactile_type}' not allowed. Allowed tactile types are: '{self._ALLOWED_TACTILES}'")
            exit(0)


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Migrate sr-config to sr_hand_config')
    parser.add_argument('--side',
                        type=str,
                        help='side of the hand (left or right)',
                        choices=['left', 'right'],
                        required=True)
    
    parser.add_argument('--hand_version',
                        type=str,
                        help='hand version (e.g. E3M5)',
                        choices=['E3M5', 'G1M5', 'E4'],
                        required=True)
    
    parser.add_argument('--tactile_type',
                        type=str,
                        help='tactile type (e.g. pst, bt_sp)',
                        choices=['bt_sp', 'bt_2p', 'pst', 'mst'],
                        required=True)
    
    parser.add_argument('--hand_serial',
                        type=int,
                        help='hand serial number (e.g. 4106)',
                        required=True)
    
    parser.add_argument('--sr_config_branch',
                        type=str,
                        help='sr-config branch to migrate from',
                        required=True)
    
    args = parser.parse_args()
    side = args.side
    hand_version = args.hand_version
    tactile_type = args.tactile_type
    hand_serial = args.hand_serial
    sr_config_branch = args.sr_config_branch
    
    # side = 'left'
    # hand_version = 'E3M5'
    # tactile_type = 'bt_sp'
    # hand_serial = '41061'
    # sr_config_branch = 'demohand_E'


    sr_config_migrator = SrConfigMigrator(side, hand_version, tactile_type, hand_serial, sr_config_branch)

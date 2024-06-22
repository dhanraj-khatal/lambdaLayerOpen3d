class Constants:
    # VERBOSE DETAILS:
    #   0: No extra detail
    #   1: Show print statements
    #   2: Show open 3d
    GUMLINE_NEIGHBOURING_DISTANCE_THRESHOLD = 1
    MAX_TEETH_COUNT = 32
    CREATE_TEETH_STEPS = True
    VERBOSE_DETAILS = 0
    DECIMAL_PRECISION = 4

    CREATE_VIDEO = True
    DISTANCE_MOVEMENT_THRESHOLD = 0.25
    SAME_JAW_COLLISION_THRESHOLD = -2
    OTHER_JAW_COLLISION_THRESHOLD = -3
    PRE_COLLISION_CHECK_DELTA = 0.7
    ANGLE_ROTATION_THRESHOLD = 2
    KDTREE_DIAMETER = 1

    EPSILON = 0.01

    PERFORM_CENTER_CROWN_TOP_SLOPE_MATCHING = False
    PERFORM_CROWN_SLOPE_MATCHING = True
    PERFORM_CROWN_FRONT_POINT_SLOPE_MATCHING = True

    SHOW_INTERMEDIATE_ANGLE_PLOTS = False
    SHOW_GEOMETRIES = False

    # FOLDER CONSTANTS
    BASE_PATH = "sample_cases/"

    INITIAL_CROWDING_CASES_BASE_PATH = "crowding_cases/"
    FINAL_CASES_BASE_PATH = "processed_crowding_final_cases/"
    PROCESSED_SAMPLE_CASES_PATH = "processed_sample_cases/"

    TEETH_FILE_NAME = "Tooth_"

    # SAMPLE CASES CONSTANTS
    SAMPLE_CASES = [
        "angela/initial/",
        "abhijeet/initial/",
        "anandita/initial/",
        "aquib/initial/",
        "ashok/initial/",
        "girish/initial/",
        "haruka/initial/",
        "hazarika/initial/",
        "madhavi/initial/",
        "purvesh/initial/",
        "edges_curved/",
        "roots_stubbed/"
    ]

    CROWDING_CASES = [
        'no_folder/',  # dummy
        'sanjay/',
        'kavita/',
        'inout/',
        'ashita/',
        'mansi/',
    ]

    DUMMY_CASE = 'no_folder/'

    FULL_CROWDING_CASES = [
        'ashita/',
        'inout/',
        'mansi/',
        'rutzu/',
        'simran/',
        'shivika/'
        'stephanie/',

        'santosh/',

        # missing root json file for Tooth 19
        # 'suraj/',

        # missing json file for Tooth 10
        # 'sanjay/',

        # wrong axes for few teeth
        # 'kavita/',
        # 'sumedha/',

        # 'usha/',
        # 'tara/',
        # 'torres/',
    ]

    PROCESSED_SAMPLE_CASES = [
        'sudhir_processed/',
        'girish_processed/',
        'sanjay_processed/',
        'astha_processed/',
        'aarti_processed/',
        'deeptej_processed/',
        'sumedha_processed/',
        'rupesh_processed/',
        'stephanie_processed/',
        'tara_processed/',
        'anangha_processed/',
        'ayushma_processed/',
        'rutzu_processed/',
        'simran_processed/',
        'sai_processed/',
        'suraj_processed/',
        'ashita_processed/',
        'shivika_processed/',
    ]

    AVERAGE_CASES_PATH = './final_cases/'
    CASES_FOR_AVERAGE_BITE_GRAPH = [
        'astha/',
        # 'amal/',
        # 'girish/',
        # 'rutzu/',
        # 'sai/',
        # 'anangha_processed/',
        # 'astha_processed/',
        # 'rutzu_processed_rakesh/',
        # 'girish_processed/',
        # 'sai_processed/',


        # 'sumedha_processed/',
        # 'stephanie_processed/',

        # MISSING 4, 13, 20, 29
        # 'aarti_processed/',

        # WRONG AXIS FOR A CANINE
        # 'tara_processed/',
    ]

    # TEETH CONSTANTS : 1-based
    LOWER_IDX = 1
    UPPER_IDX = 33

    # all ranges are 0-based
    MAXILLARY_RANGE = range(16)
    MANDIBULAR_RANGE = range(16, 32)

    ONLY_MOLARS_RANGE = list(range(3)) + list(range(13, 19)) + list(range(29, 32))
    MOLAR_RANGE = list(range(5)) + list(range(11, 21)) + list(range(27, 32))
    # MOLAR_RANGE = list(range(6)) + list(range(10, 22)) + list(range(26, 32))
    # MOLAR_RANGE = list(range(6)) + list(range(10, 32))
    INCISORS_RANGE = list(range(5, 11)) + list(range(21, 27))
    MANDIBULAR_INCISORS_RANGE = range(22, 26)
    MAXILLARY_INCISORS_RANGE = range(5, 11)
    MANDIBULAR_MIDDLE_INCISORS_RANGE = range(23, 25)  # 23, 24
    MAXILLARY_MIDDLE_INCISORS_RANGE = range(7, 9)  # 7, 8

    MIDDLE_INCISORS_MAPPING = {
        6: 25,
        7: 24,
        8: 23,
        9: 22,
    }

    INCISORS_MAPPING = {
        0: 31,
        1: 30,
        2: 29,
        3: 28,
        4: 27,
        5: 26,
        # 6: 25,
        # 7: 24,
        # 8: 23,
        # 9: 22,
        10: 21,
        11: 20,
        12: 19,
        13: 18,
        14: 17,
        15: 16,
    }

    # AXIS CONSTANTS
    AXIS_EXTEND_FACTOR = 5

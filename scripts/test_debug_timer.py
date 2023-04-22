from bimanual.utils.debug_utils import DebugTimer


if __name__ == "__main__":
    with DebugTimer(return_time_flag =True) as d:
        x = 5    
    
    from IPython import embed; embed()
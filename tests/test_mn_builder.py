from colinker.mn_builder import builder

mn = builder('mn_pv_1_2_bess.json')
mn.interSecureModelNetwork()
mn.setup()
mn.start()
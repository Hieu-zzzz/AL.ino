import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
delivery_time = ctrl.Antecedent(np.arange(0, 10.1, 0.1), 'delivery_time')  # 0=nhanh, 10=rất chậm
product_quality = ctrl.Antecedent(np.arange(0, 10.1, 0.1), 'product_quality')
cost = ctrl.Antecedent(np.arange(0, 10.1, 0.1), 'cost')
suitability = ctrl.Consequent(np.arange(0, 10.1, 0.1), 'suitability')
# Delivery Time: Fast / Moderate / Slow
delivery_time['fast']     = fuzz.trapmf(delivery_time.universe, [0, 0, 2, 4])
delivery_time['moderate'] = fuzz.trimf(delivery_time.universe, [2, 5, 8])
delivery_time['slow']     = fuzz.trapmf(delivery_time.universe, [6, 8, 10, 10])
# Product Quality: Poor / Average / Good
product_quality['poor']    = fuzz.trapmf(product_quality.universe, [0, 0, 2.5, 4.5])
product_quality['average'] = fuzz.trimf(product_quality.universe, [3, 5, 7])
product_quality['good']    = fuzz.trapmf(product_quality.universe, [5.5, 7.5, 10, 10])
# Cost: Low / Medium / High
cost['low']    = fuzz.trapmf(cost.universe, [0, 0, 2.5, 4.5])
cost['medium'] = fuzz.trimf(cost.universe, [3, 5, 7])
cost['high']   = fuzz.trapmf(cost.universe, [5.5, 7.5, 10, 10])
delivery_time.view()
product_quality.view()
cost.view()
# Output: Supplier Suitability: Low / Medium / High
suitability['low']    = fuzz.trapmf(suitability.universe, [0, 0, 2.5, 4.5])
suitability['medium'] = fuzz.trimf(suitability.universe, [3, 5, 7])
suitability['high']   = fuzz.trapmf(suitability.universe, [5.5, 7.5, 10, 10])
suitability.view()
suitability.defuzzify_method = 'centroid'
r1  = ctrl.Rule(product_quality['poor']    & delivery_time['fast']     & cost['low'],    suitability['medium'])
r2  = ctrl.Rule(product_quality['poor']    & delivery_time['fast']     & cost['medium'], suitability['medium'])
r3  = ctrl.Rule(product_quality['poor']    & delivery_time['fast']     & cost['high'],   suitability['low'])

r4  = ctrl.Rule(product_quality['poor']    & delivery_time['moderate'] & cost['low'],    suitability['medium'])
r5  = ctrl.Rule(product_quality['poor']    & delivery_time['moderate'] & cost['medium'], suitability['low'])
r6  = ctrl.Rule(product_quality['poor']    & delivery_time['moderate'] & cost['high'],   suitability['low'])

r7  = ctrl.Rule(product_quality['poor']    & delivery_time['slow']     & cost['low'],    suitability['low'])
r8  = ctrl.Rule(product_quality['poor']    & delivery_time['slow']     & cost['medium'], suitability['low'])
r9  = ctrl.Rule(product_quality['poor']    & delivery_time['slow']     & cost['high'],   suitability['low'])

r10 = ctrl.Rule(product_quality['average'] & delivery_time['fast']     & cost['low'],    suitability['high'])
r11 = ctrl.Rule(product_quality['average'] & delivery_time['fast']     & cost['medium'], suitability['medium'])
r12 = ctrl.Rule(product_quality['average'] & delivery_time['fast']     & cost['high'],   suitability['medium'])

r13 = ctrl.Rule(product_quality['average'] & delivery_time['moderate'] & cost['low'],    suitability['medium'])
r14 = ctrl.Rule(product_quality['average'] & delivery_time['moderate'] & cost['medium'], suitability['medium'])
r15 = ctrl.Rule(product_quality['average'] & delivery_time['moderate'] & cost['high'],   suitability['medium'])

r16 = ctrl.Rule(product_quality['average'] & delivery_time['slow']     & cost['low'],    suitability['medium'])
r17 = ctrl.Rule(product_quality['average'] & delivery_time['slow']     & cost['medium'], suitability['medium'])
r18 = ctrl.Rule(product_quality['average'] & delivery_time['slow']     & cost['high'],   suitability['low'])

r19 = ctrl.Rule(product_quality['good']    & delivery_time['fast']     & cost['low'],    suitability['high'])
r20 = ctrl.Rule(product_quality['good']    & delivery_time['fast']     & cost['medium'], suitability['high'])
r21 = ctrl.Rule(product_quality['good']    & delivery_time['fast']     & cost['high'],   suitability['high'])

r22 = ctrl.Rule(product_quality['good']    & delivery_time['moderate'] & cost['low'],    suitability['high'])
r23 = ctrl.Rule(product_quality['good']    & delivery_time['moderate'] & cost['medium'], suitability['high'])
r24 = ctrl.Rule(product_quality['good']    & delivery_time['moderate'] & cost['high'],   suitability['medium'])

r25 = ctrl.Rule(product_quality['good']    & delivery_time['slow']     & cost['low'],    suitability['high'])
r26 = ctrl.Rule(product_quality['good']    & delivery_time['slow']     & cost['medium'], suitability['medium'])
r27 = ctrl.Rule(product_quality['good']    & delivery_time['slow']     & cost['high'],   suitability['medium'])
system = ctrl.ControlSystem([r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27])
sim = ctrl.ControlSystemSimulation(system)
#ĐÁNH GIÁ
sim.input['delivery_time'] = 9
sim.input['product_quality'] = 7
sim.input['cost'] = 9
sim.compute()
print(sim.output['suitability'])
suitability.view(sim=sim)

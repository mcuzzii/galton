from manimlib.imports import *


def add(v1, v2):
    return [v1[0] + v2[0], v1[1] + v2[1]]


def mul(v, s):
    return [v[0] * s, v[1] * s]


def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]


def squared_distance(a, b):
    return (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1])


class AABB:
    def __init__(self, center, width, height):
        self.center = center
        self.width = width
        self.height = height

    def contains(self, mob):
        return self.center[0] - self.width <= mob.pos[0] < self.center[0] + self.width and self.center[1] \
               - self.height <= mob.pos[1] < self.center[1] + self.height

    def intersects(self, rect):
        return not (self.center[0] - self.width >= rect.center[0] + rect.width or self.center[0] + self.width <=
                    rect.center[0] - rect.width or self.center[1] + self.height <= rect.center[1] - rect.height or
                    self.center[1] - self.height >= rect.center[1] + rect.height)

    def encompass(self, rect):
        return self.center[0] - self.width <= rect.center[0] - rect.width and self.center[1] - self.height <= \
               rect.center[1] - rect.height and rect.center[0] + rect.width <= self.center[0] + self.width and \
               rect.center[1] + rect.height <= self.center[1] + self.height


class QuadTree:
    def __init__(self, boundary, cap):
        self.boundary = boundary
        self.cap = cap
        self.array = []
        self.divided = False

    def subdivide(self):
        x = self.boundary.center[0]
        y = self.boundary.center[1]
        w = self.boundary.width / 2
        h = self.boundary.height / 2
        self.northwest = QuadTree(AABB((x - w, y + h, 0), w, h), self.cap)
        self.northeast = QuadTree(AABB((x + w, y + h, 0), w, h), self.cap)
        self.southeast = QuadTree(AABB((x + w, y - h, 0), w, h), self.cap)
        self.southwest = QuadTree(AABB((x - w, y - h, 0), w, h), self.cap)
        self.divided = True

    def insert(self, mob):
        if not self.boundary.contains(mob):
            return
        if len(self.array) < self.cap:
            self.array.append(mob)
        else:
            if not self.divided:
                self.subdivide()
            self.northwest.insert(mob)
            self.northeast.insert(mob)
            self.southeast.insert(mob)
            self.southwest.insert(mob)

    def query(self, rect):
        found = []
        if not self.boundary.intersects(rect):
            return found
        if rect.encompass(self.boundary):
            found.extend(self.array)
        else:
            for p in self.array:
                if rect.contains(p):
                    found.append(p)
        if self.divided:
            found.extend(self.northwest.query(rect))
            found.extend(self.northeast.query(rect))
            found.extend(self.southeast.query(rect))
            found.extend(self.southwest.query(rect))
        return found


class Particle(Dot):
    def __init__(self, position_init, vector_init, bounce_init=0.5, **kwargs):
        self.pos = position_init
        self.vector = vector_init
        self.bounce_coeff = bounce_init
        self.category = 0
        self.funnelled = False
        self.airborne = True
        Dot.__init__(self, point=np.array(position_init + [0]), **kwargs)


class scene_1(Scene):
    CONFIG = {
        "runtime": 30,
        "particle_amount": 1000,
        "gravity": -9.8,
        "steps_per_frame": 10,
        "obstacle_rows": 17,
        "obstacle_init_columns": 8,
        "obstacle_radius": DEFAULT_SMALL_DOT_RADIUS * 1.3,
        "obs_color": "#888888",
        "p_radius": DEFAULT_SMALL_DOT_RADIUS
    }

    def construct(self):
        self.particles = VGroup()
        self.obstacles = VGroup()
        self.initialize()
        self.add(self.particles, self.obstacles)
        self.wait(self.runtime)

    def initialize(self):
        random.seed()
        self.obstacles.add(Arc(start_angle=0, angle=PI / 4, stroke_width=3, radius=1, color=self.obs_color),
                           Arc(start_angle=PI, angle=-PI / 4, stroke_width=3, radius=1, color=self.obs_color))
        self.obstacles[0].move_arc_center_to((-2 * self.p_radius - 1, 3.35, 0))
        self.obstacles[1].move_arc_center_to((2 * self.p_radius + 1, 3.35, 0))
        dist = 2 * self.obstacle_radius + 3 * self.p_radius
        height = dist * 1.732 / 2
        for i in range(self.obstacle_rows):
            for j in range(self.obstacle_init_columns + i):
                self.obstacles.add(Particle([dist * (j - (self.obstacle_init_columns + i - 1) / 2), 3.15 - i * height],
                                            0, color=self.obs_color, radius=self.obstacle_radius))
        last_row_columns = self.obstacle_init_columns + self.obstacle_rows - 1
        num_obstacles = int(self.obstacle_rows * (self.obstacle_init_columns + last_row_columns) / 2)
        x_barriers = []
        for i in range(2 + num_obstacles - last_row_columns, 2 + num_obstacles):
            line_1 = Line((self.obstacles[i].pos[0] - self.obstacle_radius * 0.75,
                           self.obstacles[i].pos[1] - dist, 0), (self.obstacles[i].pos[0] -
                                                                          self.obstacle_radius * 0.75, -5, 0),
                          color=self.obs_color, stroke_width=1, stroke_opacity=0.5)
            line_2 = Line((self.obstacles[i].pos[0] + self.obstacle_radius * 0.75,
                           self.obstacles[i].pos[1] - dist, 0), (self.obstacles[i].pos[0] +
                                                                          self.obstacle_radius * 0.75, -5, 0),
                          color=self.obs_color, stroke_width=1, stroke_opacity=0.5)
            arc_1 = ArcBetweenPoints(np.array([self.obstacles[i].pos[0], self.obstacles[i].pos[1] -
                                               dist / 3, 0]), np.array([self.obstacles[i].pos[0] -
                                                                        self.obstacle_radius * 0.75,
                                                                        self.obstacles[i].pos[1] - dist, 0]),
                                     PI * 5 / 36, color=self.obs_color, stroke_width=1, stroke_opacity=0.5)
            arc_2 = ArcBetweenPoints(np.array([self.obstacles[i].pos[0] + self.obstacle_radius * 0.75,
                                               self.obstacles[i].pos[1] - dist, 0]),
                                     np.array([self.obstacles[i].pos[0], self.obstacles[i].pos[1] -
                                               dist / 3, 0]), PI * 5 / 36, color=self.obs_color, stroke_width=1,
                                     stroke_opacity=0.5)
            black = Rectangle(width=1.5 * self.p_radius, height=self.obstacles[i].pos[1] - dist + 5,
                              fill_opacity=1, fill_color=BLACK, stroke_opacity=0)
            black.move_to((self.obstacles[i].pos[0], self.obstacles[i].pos[1] - black.get_height() /
                           2 - 0.25, 0))
            x_barriers.append(self.obstacles[i].pos[0])
            self.add(black, line_1, line_2, arc_1, arc_2)
        decorations = VGroup()
        ref_point = np.array(self.obstacles[2 + num_obstacles - last_row_columns].pos + [0])
        decorations.add(FunctionGraph(lambda x: 1.732 * (x - ref_point[0] + dist) + ref_point[1] - 0.05,
                                      x_min=ref_point[0] - dist + 0.1, stroke_width=1, color=self.obs_color,
                                      stroke_opacity=0.5))
        decorations.add(ArcBetweenPoints(ref_point + np.array([-dist + 0.1, 0.1 * 1.732 - 0.05, 0]), ref_point +
                                         np.array([-dist, -0.25, 0]), PI / 6, stroke_width=1, color=self.obs_color,
                                         stroke_opacity=0.5))
        decorations.add(Line(ref_point + np.array([-dist, -0.25, 0]), ref_point + np.array([-dist, -0.25, 0]) + DOWN
                             * 5, stroke_width=1, color=self.obs_color, stroke_opacity=0.5))
        decorations.add(Line(ref_point + np.array([-1.5 * dist, 5, 0]), ref_point + np.array([-1.5 * dist, -2.05, 0]),
                             color=self.obs_color))
        decorations.add(ArcBetweenPoints(ref_point + np.array([-1.5 * dist - 2, -4.05, 0]), ref_point +
                                         np.array([-1.5 * dist, -2.05, 0]), PI / 2, color=self.obs_color))
        decorations.add(Line(ref_point + np.array([-dist, 0.2, 0]), ref_point + LEFT * dist + UP * 5,
                             stroke_width=1, color=self.obs_color, stroke_opacity=0.5))
        decorations.add(ArcBetweenPoints(ref_point + np.array([-dist, 0.2, 0]), ref_point +
                                         np.array([-dist + 7 * (1 - 1.732 / 2), 4, 0]), PI / 6, stroke_width=1,
                                         color=self.obs_color, stroke_opacity=0.5).shift(UP * 1.732 + RIGHT))
        decorations.add(Line(ref_point + np.array([-dist, 0.2, 0]), ref_point +
                             np.array([-dist + 1, 0.2 + 1.732, 0]), stroke_width=1, color=self.obs_color,
                             stroke_opacity=0.5))
        self.add(decorations)
        self.add(decorations.copy().flip(UP).move_to(decorations.get_center() -
                                                     RIGHT * 2 * decorations.get_center()[0]))
        self.counter = 0

        def updater(mob, dt):
            if self.counter < self.particle_amount:
                particle_colors = [MAROON_A, MAROON_B, MAROON_C, MAROON_D, MAROON_E]
                mob.add(Particle([-(0.28 + self.p_radius) * (1 - 2 * random.random()), 4], [0, self.gravity / 60],
                                 radius=self.p_radius, color=particle_colors[random.randint(0, 4)]))
                self.counter += 1
            for i in range(self.counter):
                mob[i].move_to(np.array(mob[i].pos + [0]))
                if mob[i].pos[1] >= 3.15 and not mob[i].funnelled:
                    arc_center = self.obstacles[math.ceil(mob[i].pos[0])].get_arc_center().tolist()
                    delta = squared_distance(arc_center, mob[i].pos)
                    if delta <= (self.p_radius + 1) * (self.p_radius + 1):
                        delta = math.sqrt(delta)
                        unit_collision = mul(add(arc_center, mul(mob[i].pos, -1)), 1 / delta)
                        mob[i].vector = add(mob[i].vector, mul(unit_collision, -(1 + mob[i].bounce_coeff) *
                                                               dot(mob[i].vector, unit_collision)))
                        mob[i].pos = add(mob[i].pos, mul(unit_collision, delta - 1 - self.p_radius))
                else:
                    mob[i].funnelled = True
                    if mob[i].pos[1] <= ref_point[1]:
                        if mob[i].category == 0:
                            mob[i].category = (mob[i].pos[0] - ref_point[0]) / dist
                    x_thres = -(mob[i].pos[1] - ref_point[1] + 0.05) / 1.732 - ref_point[0] + dist - 2 * \
                              self.p_radius / 1.732
                    if not abs(mob[i].pos[0]) < x_thres:
                        index = 2 * (mob[i].pos[0] > 0) - 1
                        unit_collision = [index * 1.732 / 2, 1 / 2]
                        mob[i].vector = add(mob[i].vector, mul(unit_collision, -2 * dot(mob[i].vector,
                                                                                        unit_collision)))
                        mob[i].pos[0] -= index * (abs(mob[i].pos[0]) - x_thres)
                    x_ref = self.obstacles[2].pos[0]
                    y_ref = self.obstacles[2].pos[1]
                    if y_ref + height / 2 > mob[i].pos[1] >= y_ref - height * (self.obstacle_rows - 1 / 2):
                        row = math.ceil((y_ref + height / 2 - mob[i].pos[1]) / height)
                        if x_ref - row * dist / 2 < mob[i].pos[0] <= x_ref + dist * ((self.obstacle_init_columns - 1)
                                                                                     + row / 2):
                            column = math.ceil((mob[i].pos[0] - x_ref + row * dist / 2) / dist)
                            max_columns = self.obstacle_init_columns + row - 1
                            index = 1 + int(row * (self.obstacle_init_columns + max_columns) / 2) - max_columns \
                                    + column
                            ob_center = self.obstacles[index].pos
                            delta = squared_distance(ob_center, mob[i].pos)
                            threshold = self.p_radius + self.obstacle_radius
                            if delta <= threshold * threshold:
                                delta = math.sqrt(delta)
                                unit_collision = mul(add(ob_center, mul(mob[i].pos, -1)), 1 / delta)
                                mob[i].vector = add(mob[i].vector, mul(unit_collision, -(1 + mob[i].bounce_coeff) *
                                                                       dot(mob[i].vector, unit_collision)))
                                mob[i].pos = add(mob[i].pos, mul(unit_collision, delta - self.p_radius -
                                                                 self.obstacle_radius))
            for j in range(self.steps_per_frame):
                ptree = QuadTree(AABB(ORIGIN, FRAME_HEIGHT / 2, FRAME_HEIGHT / 2), 2)
                for i in range(self.counter):
                    if mob[i].airborne:
                        mob[i].vector[1] += self.gravity * dt / self.steps_per_frame
                    mob[i].pos = add(mob[i].pos, mul(mob[i].vector, dt / self.steps_per_frame))
                    mob[i].airborne = True
                    if mob[i].pos[1] <= ref_point[1] + 0.2:
                        if not abs(mob[i].pos[0]) <= dist - ref_point[0] - self.p_radius:
                            index = 2 * (mob[i].pos[0] > 0) - 1
                            mob[i].vector[0] *= -mob[i].bounce_coeff
                            mob[i].pos[0] = index * (dist - ref_point[0] - self.p_radius)
                        if not mob[i].category == 0:
                            if mob[i].category >= 0:
                                x = x_barriers[min(last_row_columns - 1, math.floor(mob[i].category))] + 0.75 * \
                                    self.obstacle_radius
                                if mob[i].pos[0] - self.p_radius <= x:
                                    mob[i].vector[0] *= -mob[i].bounce_coeff
                                    mob[i].pos[0] = x + self.p_radius
                            if mob[i].category <= last_row_columns - 1:
                                x = x_barriers[max(0, math.ceil(mob[i].category))] - 0.75 * self.obstacle_radius
                                if mob[i].pos[0] + self.p_radius >= x:
                                    mob[i].vector[0] *= -mob[i].bounce_coeff
                                    mob[i].pos[0] = x - self.p_radius
                    if mob[i].pos[1] - self.p_radius < -FRAME_HEIGHT / 2:
                        mob[i].vector[1] *= -mob[i].bounce_coeff / 3
                        mob[i].pos[1] = self.p_radius - FRAME_HEIGHT / 2
                        mob[i].airborne = False
                    ptree.insert(mob[i])
                for i in range(self.counter):
                    neighbors = ptree.query(AABB(mob[i].pos, 2 * self.p_radius + 0.001, 2 * self.p_radius + 0.001))
                    for m in neighbors:
                        if mob[i].pos == m.pos:
                            continue
                        delta = squared_distance(mob[i].pos, m.pos)
                        if delta < 4 * self.p_radius * self.p_radius:
                            delta = math.sqrt(delta)
                            unit_collision = mul(add(m.pos, mul(mob[i].pos, -1)), 1 / delta)
                            error = self.p_radius - delta / 2 + 0.001
                            v_2 = dot(unit_collision, mob[i].vector)
                            v_1 = dot(unit_collision, m.vector)
                            mob[i].pos = add(mob[i].pos, mul(unit_collision, -error))
                            m.pos = add(m.pos, mul(unit_collision, error))
                            mob[i].vector = add(mob[i].vector, mul(unit_collision, v_1 - v_2))
                            m.vector = add(m.vector, mul(unit_collision, v_2 - v_1))
                            if not mob[i].category == 0:
                                mob[i].airborne = False
                                m.airborne = False

        self.particles.add_updater(updater)

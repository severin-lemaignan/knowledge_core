// import * as d3 from 'https://cdn.skypack.dev/d3@7';
import {Mutex, MutexInterface, Semaphore, SemaphoreInterface, withTimeout} from 'async-mutex';
import * as d3 from 'd3';
import ROSLIB from 'roslib';

const ICON_MAP = {
    'owl:Thing': 'GenericInstance',
    'cyc:Plant': 'leaf',
    'dbr:Plant': 'leaf',
    'dbr:Ornamental_plant': 'leaf',
    'cyc:Color': 'invert-colors',
    'cyc:EnduringThing-Localized': 'map',
    'cyc:SpatialThing-Localized': 'map-marker',
    'cyc:SpatialThing': 'record-circle-outline',
    'cyc:SpaceRegion': 'selection-marker',
    'undecided': 'progress-question',
    'Robot': 'Robot',
    'Human': 'Human',
    'foaf:Person': 'Human',
    'cyc:TemporalThing': 'Time',
    'cyc:Event': 'timer-outline',
    'EmbodiedAgent': 'Agent',
    'Toolbox': 'toolbox-outline',
    'Agent': 'Agent',
    'foaf:Agent': 'Agent',
    'Box': 'GenericInstance',
    'Cardboardbox': 'GenericInstance',
    'cyc:FurniturePiece': 'Furniture',
    'Table': 'Furniture',
    'dbr:Table_(furniture)': 'Furniture',
    'dbr:Coffee_table': 'Furniture',
    'cyc:CellularTelephone': 'cellphone',
    'dbr:Smartphone': 'cellphone',
    'Cloth': 'tshirt-crew',
    'Trashbin': 'trash-can',
    'dbr:Waste_container': 'trash-can',
    'Tableware': 'silverware-fork-knife',
    'dbr:Tableware': 'silverware-fork-knife',
    'dbr:Fork': 'silverware-fork-knife',
    'dbr:Knife': 'silverware-fork-knife',
    'dbr:Spoon': 'silverware-fork-knife',
    'dbr:Kitchen_utensil': 'silverware-fork-knife',
    'Window': 'window-closed-variant',
    'GraspableObject': 'hand',
    'Book': 'book-open-variant',
    'dbr:Book': 'book-open-variant',
    'cyc:Chair-PieceOfFurniture': 'chair-rolling',
    'dbr:Chair': 'chair-rolling',
    'cyc:Sofa-PieceOfFurniture': 'sofa-single',
    'dbr:Couch': 'sofa-single',
    'cyc:ArmChair': 'armchair',
    'dbr:Armchair_(furniture)': 'armchair',
    'Remote': 'remote-tv',
    'cyc:Tray': 'tray',
    'Bottle': 'bottle-wine',
    'dbr:Bottle': 'bottle-wine',
    'Glass': 'cup-water',
    'dbr:Glass_(drinkware)': 'cup-water',
    'Cup': 'cup-water',
    'dbr:Cup': 'cup-water',
    'Placemat': 'mat',
    'Container': 'contain',
    'cyc:Action': 'run-fast',
    'cyc:Situation': 'location-enter',
    'cyc:PurposefulAction': 'target',
    'Rest': 'chat-sleep',
    'Manipulation': 'hand',
    'TaskPlan': 'chart-gantt',
    'TimeInterval': 'timelapse',
    'ContinuousTimeInterval': 'timelapse',
    'TimePoint': 'timeline',
    'Avatar': 'account-box-outline',
    'cyc:Hand': 'hand',
    'Hand': 'hand',
    'Torso': 'torso',
    'BodyPart': 'human-handsup',
    'Eye': 'eye',
    'Head': 'head',
    'Shelf': 'bookshelf',
    'dbr:Shelf_(storage)': 'bookshelf',
    'cyc:Object-SupportingFurniture': 'support',
    'cyc:FluidTangibleThing': 'water',
    'cyc:EdibleStuff': 'food-apple',
    'dbr:Apple': 'food-apple',
    'dbr:Pear': 'food-pear',
    'Artifact': 'hammer',
    'cyc:PartiallyTangible': 'tangible',

}

const t = d3.transition().duration(750).ease(d3.easeLinear);

type NodeType =
    'class'|'instance'|'object_property'|'datatype_property'|'undecided';

type Link = {
    source: string; target: string;
    label?: string;
    dist?: number;
}

type AdjencyList = Record<string, Node[]>;

class RosKBClient {
  private ros: ROSLIB.Ros;
  private details_service: ROSLIB.Service;
  private active_concepts_sub: ROSLIB.Topic;

  constructor(rosBridgeUrl: string) {
    this.ros = new ROSLIB.Ros({
      url: rosBridgeUrl,
    });

    this.ros.on('connection', () => {
      console.log('Connected to rosbridge server.');
    });

    this.ros.on('error', (error) => {
      console.error('Error connecting to rosbridge server:', error);
    });

    this.ros.on('close', () => {
      console.log('Connection to rosbridge server closed.');
    });

    this.details_service = new ROSLIB.Service({
      ros: this.ros,
      name: '/kb/details',
      serviceType: 'kb_msgs/srv/About',
    });

    this.active_concepts_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/kb/active_concepts',
      messageType: 'kb_msgs/ActiveConcepts',
    });
  }

  details(term: string): Promise<any> {
    return new Promise((resolve, reject) => {
      const requestMsg = new ROSLIB.ServiceRequest({term: term});

      this.details_service.callService(requestMsg, (response) => {
        resolve(response);
      }, (error) => {
        reject(error);
      });
    });
  }

  on_active_concepts(callback: (message: any) => void): void {
    this.active_concepts_sub.subscribe((message) => {
      callback(message);
    });
  }

}

const ros_kb_client = new RosKBClient('ws://localhost:9090');

class Node {
    selected: boolean;
    hovered: boolean;
    icon: string = '';
    highlight: boolean;
    depth: number;

    // d3 stuff
    x: number;
    y: number;
    fx: number;
    fy: number;


    constructor(
        public readonly id: string,
        public readonly type: NodeType,
        public readonly classes: string[] = [],
        public readonly instances: string[] = [],
        public readonly superClasses: string[] = [],
        public readonly subClasses: string[] = [],
        public readonly rels: [string, string, string][] = [],
        public readonly label: string = id,
    ) {
        this.selected = false;
        this.hovered = false;
        this.highlight = false;
        this.depth = 0;

    }

    /* sets a very low depth for all nodes connected to this node, except for
     * selected nodes.
     */
    public resetDepths() {
        const graph = asAdjencyList(KB);
        // pre-populated visited with 'false' for all nodes
        let visited = Object.fromEntries(
            Object.keys(graph).map((k: string) => [k, false]));

        this.selected ? this.depth = 0 : this.depth = 5000;

        // Use an array as our queue representation:
        let q: string[] = new Array<string>();

        visited[this.id] = true;

        q.push(this.id);

        while (q.length > 0) {
            const node_id = q.pop();
            for (let adjN of graph[node_id]) {
                if (!visited[adjN.id]) {
                    visited[adjN.id] = true;

                    // /!\ CURRENTLY, unselect all other nodes
                    adjN.selected = false;
                    adjN.fx = null;
                    adjN.fy = null;

                    if (!adjN.selected) {
                        adjN.depth = 10000;
                    } else {
                        adjN.depth = 0;
                    }
                    q.push(adjN.id);
                }
            }
        }
    }

    public updateDepths(reset_depth: boolean = true) {
        const graph = asAdjencyList(KB);
        // pre-populated visited with 'false' for all nodes
        let visited = Object.fromEntries(
            Object.keys(graph).map((k: string) => [k, false]));

        if (reset_depth) {
            this.resetDepths();
        }

        // Use an array as our queue representation:
        let q: [string, number][] = new Array<[string, number]>();

        visited[this.id] = true;

        q.push([this.id, this.depth]);

        while (q.length > 0) {
            const [node_id, depth] = q.pop();
            for (let adjN of graph[node_id]) {
                if (!visited[adjN.id]) {
                    visited[adjN.id] = true;

                    // unselected all other nodes
                    adjN.selected = false;

                    if (adjN.depth > depth + 1) {
                        if (!adjN.selected) {
                            adjN.setDepth(depth + 1);
                            q.push([adjN.id, depth + 1]);
                        } else {
                            adjN.setDepth(0)
                            adjN.updateDepths(false);
                        }
                    }
                }
            }
        }
    }

    public setDepth(d: number) {
        this.depth = d;
        // console.log(`${this.id}: depth = ${this.depth}`);
    }

    public static async fromKB(term: string): Promise<Node> {
        let attrs = {
            'superClasses': [],
            'subClasses': [],
            'classes': [],
            'instances': []
        };


        let response = await ros_kb_client.details(term);
        if (!response || !response['json']) {
            console.log("Service call 'details' failed for term " + term);
            return new Node(term, 'undecided');
        }
        let res = JSON.parse(response['json']);

        //console.log('Service call succeeded:', res);

        for (const attr of res['attributes']) {
            for (const sc of attr['values']) {
                attrs[attr['id']].push(sc['id']);
            }
        }

        return new Node(
            res['id'],
            res['type'],
            attrs['classes'],
            attrs['instances'],
            attrs['superClasses'],
            attrs['subClasses'],
            res['relations'],
            res['label']['default'],
        );
    }
}

type Graph = {
    links: Link[],
    nodes: Node[],
};

function asAdjencyList(graph: Graph): AdjencyList {
    let nodes = Object.fromEntries(graph.nodes.map(d => [d.id, d]));

    let adj = Object.fromEntries(graph.nodes.map(d => [d.id, []]));

    for (const link of graph.links) {
        adj[link.source].push(nodes[link.target]);
        adj[link.target].push(nodes[link.source]);
    }

    return adj;
}


let KB: Graph = {
    links: [],
    nodes: [
        new Node('EmbodiedAgent', 'class'),
    ]
}

let kbMutex = new Mutex();

function hasLink(link: Link): boolean {
    return (KB.links.some(
        l => l.source === link.source && l.target === link.target &&
            l.label === link.label));
}

function hasNode(id: string): boolean {
    return (KB.nodes.some(n => n.id === id));
}

async function updateTerm(term: string): Promise<Node> {
    // lock the KB to avoid concurrent modifications
    const release = await kbMutex.acquire();

    let node = await Node.fromKB(term)

    const idx = KB.nodes.findIndex(n => n.id === node.id);
    if (idx == -1) {
        console.log('Adding term ' + node.id);
        KB.nodes.push(node);
    } else {
        console.log('Updating term ' + node.id);
        KB.nodes[idx] = node;
    }

    for (const cls of node.classes) {
        let link = {source: cls, target: node.id, label: 'isA'};
        if (!hasNode(cls)) {
            console.log('Adding related term ' + cls);
            KB.nodes.push(await Node.fromKB(cls));
        }
        if (!hasLink(link)) {
            KB.links.push(link);
        }
    }

    for (const instance of node.instances) {
        let link = {source: node.id, target: instance, label: 'isA'};
        if (!hasNode(instance)) {
            console.log('Adding related term ' + instance);
            KB.nodes.push(await Node.fromKB(instance));
        }
        if (!hasLink(link)) {
            KB.links.push(link);
        }
    }

    for (const cls of node.superClasses) {
        let link = {source: cls, target: node.id, label: 'isA'};
        if (!hasNode(cls)) {
            console.log('Adding related term ' + cls);
            KB.nodes.push(await Node.fromKB(cls));
        }
        if (!hasLink(link)) {
            KB.links.push(link);
        }
    }

    for (const cls of node.subClasses) {
        let link = {source: node.id, target: cls, label: 'isA'};
        if (!hasNode(cls)) {
            console.log('Adding related term ' + cls);
            KB.nodes.push(await Node.fromKB(cls));
        }
        if (!hasLink(link)) {
            KB.links.push(link);
        }
    }

    for (const rel of node.rels) {
        let link = {source: rel[0], target: rel[2], label: rel[1]};

        // skip self-referring relations (like xyz owl:sameAs xyz)
        if (rel[0] == rel[2]) {
            continue;
        }

        // if rel[0] or rel[2] are not strings, skip the relation
        // TODO: add support for this case (ie, datatype properties)
        if (typeof rel[0] !== 'string' || typeof rel[2] !== 'string') {
            console.log('Skipping datatype property relation ' + rel[0] + ' ' + rel[1] + ' ' + rel[2]);
            continue;
        }

        if (!hasNode(rel[0])) {
            console.log('Adding related term ' + rel[0]);
            KB.nodes.push(await Node.fromKB(rel[0]));
        }
        if (!hasNode(rel[2])) {
            console.log('Adding related term ' + rel[2]);
            KB.nodes.push(await Node.fromKB(rel[2]));
        }
        if (!hasLink(link)) {
            KB.links.push(link);
        }
    }

    release();  // release the lock on the KB

    return node;
}

// icons for classes = 2 x node_radius = full size
// scale for instances:
const SCALE_FACTOR_INSTANCES = 1.5

function makeChart(data, invalidation = null) {


    // Specify the dimensions of the chart.
    const width = 928;
    const height = 680;


    ///////////////////////////////////////////////////////////
    // interaction with the knowledge base

    ros_kb_client.on_active_concepts((msg) => {
        for (const concept of msg.concepts) {
            updateTerm(concept).then(node => {
                console.log('Updated node ' + node.id);
                kb_graph.update(KB);
            });
        }
    });

    ///////////////////////////////////////////////////////////

    // The force simulation mutates links and nodes, so create a copy
    // so that re-evaluating this cell produces the same result.
    // !! commented out the copy: it should work in our case.
    // if not, need to copy not just the data with the all class
    const links = data.links.map(d => ({...d}));
    const nodes = data.nodes;  //.map(d => ({...d}));

    const node_radius = 25;

    // Create a simulation with several forces.
    const simulation =
        d3.forceSimulation(nodes)
            .force('link', d3.forceLink(links).id(d => d.id).distance(d => 50))
            .force('charge', d3.forceManyBody().strength(-1000))
            .force(
                'collide', d3.forceCollide(nodes).radius(n => node_radius * 2))
            .force('x', d3.forceX())
            .force('y', d3.forceY());

    // Create the SVG container.
    const svg = d3.create('svg')
                    .attr('width', '100vw')
                    .attr('height', '100vh')
                    .attr('viewBox', [-width / 2, -height / 2, width, height])
                    .attr('style', 'max-width: 100vw; height: 100vh;');

    const defs = svg.append('defs');
    defs.append('marker')
        .attr('id', 'arrowhead')
        .attr('markerWidth', 15)
        .attr('markerHeight', 15)
        .attr('refX', 0)
        .attr('refY', 7.5)
        .attr('orient', 'auto')
        .attr('stroke', 0)
        .attr('fill', 'context-stroke')
        .attr(
            'markerUnits',
            'userSpaceOnUse')  // set to strokeWidth to scale with stroke
        .append('polygon')
        .attr('points', '0 0, 15 7.5, 0 15');

    let link = svg.append('g')
                   .attr('stroke', '#999')
                   .attr('stroke-opacity', 0.6)
                   .selectAll('g')
                   .data(links, d => d)
                   .join(enter => linkBuilder(enter));

    let node = svg.append('g')
                   .selectAll('g')
                   .data(nodes, (d: Node) => d)
                   .join(enter => nodeBuilder(enter));

    // small trick to expand the first node displayed on the screen when
    // displaying the knowledge base
    updateTerm(nodes[0].id).then(node => {
        node.selected = true;
        node.updateDepths();
        kb_graph.update(KB);
    });

    simulation.on('tick', () => {
        link.attr(
                'opacity',
                d => {
                    let depth = Math.max(d.target.depth, d.source.depth);
                    return depth < 2 ? 1 : 1 / depth
                })
            .selectChild('path')
            .attr(
                'd',
                d => `M ${d.source.x} ${d.source.y} L  ${
                    (d.source.x + d.target.x) /
                    2} ${(d.source.y + d.target.y) / 2}  L ${d.target.x} ${
                    d.target.y}`)

        node.attr(
                'transform',
                d => `translate(${d.x - node_radius} ${d.y - node_radius})`)
            .attr('opacity', d => d.depth < 2 ? 1 : 1 / d.depth)
            .selectChild('rect')
            .attr(
                'fill',
                (d: Node) => (d.selected || d.hovered) ?
                    '#ffb280ff' :
                    (d.type == 'class' ? '#fae4d1ff' : '#fff'));


        // node.filter(d => d.depth > 2)
    });

    function linkBuilder(enter) {
        let g = enter.append('g').attr('class', 'edge-group');

        g.append('path')
            .attr('id', d => `edge-${d.source.id}-${d.target.id}`)
            .attr(
                'marker-mid',
                d => {
                    switch (d.label) {
                        case 'isA': {
                            return 'url(#arrowhead)';
                        }
                        default: {
                            return 'none';
                        }
                    }
                })
            .attr('stroke-width', d => d.dist ? 5 * 1 / d.dist : 2);

        g.filter(d => d.label !== 'isA')
            .append('text')
            .attr('text-anchor', 'middle')
            .attr('class', 'edge-label')
            .append('textPath')
            .attr('href', d => `#edge-${d.source.id}-${d.target.id}`)
            .attr('startOffset', '50%')
            .text(d => {
                switch (d.label) {
                    case 'owl:equivalentClass':
                    case 'owl:sameAs': {
                        return '=';
                    }
                    default: {
                        return d.label;
                    }
                }
            });


        return g;
    }

    function nodeBuilder(enter) {
        let g = enter.append('g').attr('class', 'node-group');

        g.append('rect')
            .attr(
                'height',
                (d: Node) => d.type == 'class' ?
                    node_radius * 2 :
                    node_radius * SCALE_FACTOR_INSTANCES)
            .attr(
                'width',
                (d: Node) => d.type == 'class' ?
                    node_radius * 2 :
                    node_radius * SCALE_FACTOR_INSTANCES)
            .attr(
                'x',
                (d: Node) => d.type == 'class' ?
                    0 :
                    node_radius * (1 - SCALE_FACTOR_INSTANCES / 2))
            .attr(
                'y',
                (d: Node) => d.type == 'class' ?
                    0 :
                    node_radius * (1 - SCALE_FACTOR_INSTANCES / 2))
            .attr('rx', 5)
            .attr('stroke-width', '0px')
            .attr('stroke', '#ffb38096');

        g.append('image')
            .attr('preserveAspectRatio', 'xMidYMid')
            .attr(
                'width',
                (d: Node) => d.type == 'class' ?
                    node_radius * 2 :
                    node_radius * SCALE_FACTOR_INSTANCES)
            .attr(
                'x',
                (d: Node) => d.type == 'class' ?
                    0 :
                    node_radius * (1 - SCALE_FACTOR_INSTANCES / 2))
            .attr(
                'y',
                (d: Node) => d.type == 'class' ?
                    0 :
                    node_radius * (1 - SCALE_FACTOR_INSTANCES / 2))
            .attr('href', d => `static/icons/${icon(d)}.svg`)
            .attr('alt', d => d.id)
            .on('click touchstart',
                (evt, d: Node) => {
                    if (!d.selected) {
                        updateTerm(d.id).then(node => {
                            node.selected = true;
                            node.fx = d.x;
                            node.fy = d.y;
                            node.updateDepths();
                            kb_graph.update(KB);
                        });
                    }
                })
            .on('mouseover',
                (evt, d) => {
                    d.hovered = true;
                })
            .on('mouseout', (evt, d) => {
                d.hovered = false;
            });

        g.append('text')
            .attr('text-anchor', 'middle')
            .attr('y', node_radius * 2 + 12)
            .attr('x', node_radius)
            .attr(
                'class',
                d =>
                    d.type == 'class' ? 'node-label class-label' : 'node-label')
            .text(d => d.label);

        g.append('title').text(d => d.id);


        // Add a drag behavior.
        g.call(d3.drag()
                   .on('start', dragstarted)
                   .on('drag', dragged)
                   .on('end', dragended));

        return g;
    }


    function icon(d: Node) {

        console.log('Getting icon for ' + d.id + " (classes: " + d.classes + ")");
        if (d.type == 'class') {
            if (d.id in ICON_MAP) {
                return ICON_MAP[d.id];
            } else {
                return 'GenericClass';
            }
        } else if (d.type == 'instance') {
            if (d.classes.length > 0 && d.classes[0] in ICON_MAP) {
                return ICON_MAP[d.classes[0]];
            } else {
                return 'GenericInstance';
            }
        } else
            return ICON_MAP['undecided'];
    }

    // Reheat the simulation when drag starts, and fix the subject position.
    function dragstarted(event) {
        if (!event.active) simulation.alphaTarget(0.3).restart();

        if (!event.subject.selected) {
            updateTerm(event.subject.id).then(node => {
                node.selected = true;
                node.fx = event.subject.x;
                node.fy = event.subject.y;
                node.updateDepths();
                kb_graph.update(KB);
            });
        } else {
            event.subject.fx = event.subject.x;
            event.subject.fy = event.subject.y;
        }
    }

    // Update the subject (dragged node) position during drag.
    function dragged(event) {
        event.subject.fx = event.x;
        event.subject.fy = event.y;
    }

    // Restore the target alpha so the simulation cools after dragging ends.
    // Unfix the subject position now that it’s no longer being dragged.
    function dragended(event) {
        if (!event.active) simulation.alphaTarget(0);
        // event.subject.fx = null;
        // event.subject.fy = null;
    }

    if (invalidation) {
        invalidation.then(() => simulation.stop());
    }

    return Object.assign(svg.node(), {

        // add an 'update' method to the graph, to dynamically add or remove
        // nodes
        update(kb: Graph) {
            // Make a shallow copy to protect against mutation, while
            // recycling old nodes to preserve position and velocity.
            const old = new Map(node.data().map(d => [d.id, d]));

            let nodes = kb.nodes
                            .map(
                                d => Object.assign(
                                    old.get(d.id) || new Node('', 'class'), d))
                            .filter(d => d.depth < 4);


            let links = kb.links.map(d => Object.assign({}, d)).filter(l => {
                let keys = nodes.map(d => d.id);
                return (keys.includes(l.source) && keys.includes(l.target));
            });

            simulation.nodes(nodes);
            simulation.force('link').links(links);
            // re-heat the simulation a little
            simulation.alpha(0.5).restart();

            node =
                node.data(nodes, d => d.id).join(enter => nodeBuilder(enter));


            link = link.data(links, d => `edge-${d.source.id}-${d.target.id}`)
                       .join(enter => linkBuilder(enter));
        }
    });
}

let kb_graph = makeChart(KB);

document.getElementById('chart').appendChild(kb_graph);

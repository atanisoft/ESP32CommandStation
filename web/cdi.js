"use strict";
var ws_pending_response = new Map();
var cdi_fields = [];
const CDI_SPACE_ACDI_USER = 251;
const CDI_SPACE_CONFIG = 253;
function parseXML(xmlString) {
    console.debug(xmlString);
    var parser = new DOMParser();
    var parsererrorNS = parser.parseFromString('INVALID', 'application/xml').getElementsByTagName("parsererror")[0].namespaceURI;
    var dom = parser.parseFromString(xmlString, 'application/xml');
    if (dom.getElementsByTagNameNS(parsererrorNS, 'parsererror').length > 0) {
        console.error(dom);
        throw new Error('Error parsing XML');
    }
    return dom;
}
function pad_hex(str, length) {
    var hex = new Array(length + 1).join('0') + str;
    return hex.substr(-length).match(/.{1,2}/g).join('.');
}
function enable_button(field) {
    console.debug('enabling:', '#btn-' + field);
    $('#btn-' + field).removeAttr('disabled');
}
function disable_button(field) {
    console.debug('disabling:', '#btn-' + field);
    $('#btn-' + field).attr('disabled', true);
}
function reset_cdi_buttons(target) {
    $(String.format('#btn-refresh-{0}', target)).removeClass('loading');
    $(String.format('#btn-save-{0}', target)).removeClass('loading');
    enable_button('refresh-' + target);
    disable_button('save-' + target);
}
function save_cdi_field(field) {
    var target = $(field).attr('id').substr(9);
    $(field).toggleClass('loading');
    var size = parseInt($('#' + target).data('size'));
    var type = $('#' + target).data('type');
    var value = $('#' + target).val();
    if (type === 'str') {
        if (value.length < size) {
            size = value.length;
        } else {
            value = value.substr(0, size);
        }
        value = value.trim();
    }
    ws_tx(JSON.stringify({
        req: 'cdi',
        ofs: parseInt($('#' + target).data('offset')),
        sz: size,
        type: type,
        tgt: target,
        val: value,
        node: $('#' + target).data('node'),
        spc: parseInt($('#' + target).data('space')),
        id: get_ws_msg_id()
    }));
}
function refresh_cdi_field(field) {
    var target = $(field).attr('id').substr(12);
    $(field).toggleClass('loading');
    ws_tx(JSON.stringify({
        req: 'cdi',
        ofs: parseInt($('#' + target).data('offset')),
        sz: parseInt($('#' + target).data('size')),
        type: $('#' + target).data('type'),
        tgt: target,
        node: $('#' + target).data('node'),
        spc: parseInt($('#' + target).data('space')),
        id: get_ws_msg_id()
    }));
}
function cdi_send_event(field) {
    var event = $('#' + field).val();
    $(field).toggleClass('loading');
    ws_tx(JSON.stringify({ req: 'event', value: event, tgt: $(field).attr('id') }));
}
function refresh_all_cdi_fields() {
    $('[id^=btn-refresh-]').each((x, field) => {
        const target = $(field).attr('id').substr(12);
        cdi_fields.push({
            key: target,
            msg: JSON.stringify({
                req: 'cdi',
                ofs: parseInt($('#' + target).data('offset')),
                sz: parseInt($('#' + target).data('size')),
                type: $('#' + target).data('type'),
                tgt: target,
                node: $('#' + target).data('node'),
                spc: parseInt($('#' + target).data('space')),
                id: get_ws_msg_id()
            })
        });
    });
    $('#olcbconfig-empty').show();
    $('#olcbconfig-content').hide();
    $('#node-cdi-status-text').text('');
    $('#node-cdi-status').show();
    download_cdi_fields().then(() => {
        $('#olcbconfig-empty').hide();
        $('#olcbconfig-content').show();
    });
}
function show_group_tab(tab) {
    var selected_tab = tab.id;
    var tab_group = selected_tab.substring(0, selected_tab.lastIndexOf('-'));
    $(String.format('[id^=rep-{0}', tab_group)).each((x, replica) => {
        $(replica).hide();
        $('#' + replica.id.substr(4)).removeClass('active');
        $('#' + replica.id.substr(4)).addClass('inactive');
    });
    $(tab).addClass('active');
    $(tab).removeClass('inactive');
    $('#rep-' + selected_tab).show();
}
async function download_cdi_fields() {
    const maxConcurrentWorkers = 10;
    var activeWorkers = 0;
    var fieldIndex = 0;
    const download_start = +new Date();
    var failures = 0;
    return new Promise(done => {
        const getNextTask = () => {
            if (activeWorkers < maxConcurrentWorkers && fieldIndex < cdi_fields.length) {
                console.debug(String.format('Workers:{0}/{1}, Index:{2}/{3}', activeWorkers, maxConcurrentWorkers, fieldIndex, cdi_fields.length));
                const msg = cdi_fields[fieldIndex].msg;
                const key = cdi_fields[fieldIndex].key;
                const start = +new Date();
                $('#node-cdi-status-text').text(String.format('Retrieving field: {0}/{1}', fieldIndex, cdi_fields.length));
                new Promise((resolve, reject) => {
                    console.debug('requesting:', key, msg);
                    ws_pending_response[key] = resolve;
                    ws_tx(msg);
                    setTimeout(() => {
                        reject(new Error('Failed to receive response after 10sec: ' + msg));
                    }, 10000);
                }).then(() => {
                    const end = +new Date();
                    console.debug(String.format('{0} completed in {1} ms', key, (end - start)));
                    delete ws_pending_response[key];
                    activeWorkers--;
                    getNextTask();
                }).catch(reason => {
                    const end = +new Date();
                    console.debug(String.format('{0} failed in {1} ms: {2}', key, (end - start), reason));
                    activeWorkers--;
                    failures++;
                    console.log(String.format('Failed to download {0}, queueing redownload', key));
                    cdi_fields.push({
                        key: key,
                        msg: JSON.stringify({
                            req: 'cdi',
                            ofs: parseInt($('#' + key).data('offset')),
                            sz: parseInt($('#' + key).data('size')),
                            type: $('#' + key).data('type'),
                            tgt: key,
                            node: $('#' + key).data('node'),
                            spc: parseInt($('#' + key).data('space')),
                            id: get_ws_msg_id()
                        })
                    });
                    getNextTask();
                });
                fieldIndex++;
                activeWorkers++;
                getNextTask();
            } else if (activeWorkers === 0 && fieldIndex === cdi_fields.length) {
                const download_end = +new Date();
                console.info(String.format('Downloaded {0} fields in {1} ms ({2} failures) with {3} concurrent requests',
                    cdi_fields.length, (download_end - download_start), failures, maxConcurrentWorkers));
                cdi_fields.splice(0, cdi_fields.length);
                done();
            }
        };
        getNextTask();
    });
}
class CdiBaseElement {
    #name;
    #description;
    #size;
    #type;
    #alias;
    #origin;
    #offset;
    #space_id;
    #nodeid;
    constructor(node, origin, space, nodeid) {
        this.#name = '';
        this.#description = '';
        this.#size = parseInt($(node).attr('size')) || 1;
        this.#type = node.nodeName;
        this.#nodeid = nodeid;
        this.#alias = Math.random().toString().replace(/\./g, '_');
        this.#origin = parseInt(origin);
        this.#offset = parseInt($(node).attr('offset')) || 0;
        this.#space_id = space;
        if (this.get_type === 'group') {
            this.#size = 0;
        } else if (this.get_type === 'eventid') {
            this.#size = 8;
            this.#type = 'evt';
        } else if (this.get_type === 'string') {
            this.#type = 'str';
        }
        $(node.children).each((x, child) => {
            if (child.nodeName === 'name') {
                this.#name = $(child).text();
            } else if (child.nodeName === 'description') {
                this.#description = $(child).text();
            }
        });
        if (!this.has_name && this.has_description) {
            this.#name = this.#description;
            this.#description = '';
        }
    }
    get get_type() {
        return this.#type;
    }
    get get_name() {
        return this.#name;
    }
    get has_name() {
        return this.#name !== '';
    }
    get get_description() {
        return this.#description;
    }
    get has_description() {
        return this.#description !== '';
    }
    get get_alias() {
        return this.#alias;
    }
    get get_space() {
        return this.#space_id;
    }
    get get_offset() {
        return this.#offset;
    }
    get get_address() {
        return this.get_offset + this.get_origin;
    }
    get get_origin() {
        return this.#origin;
    }
    get get_field_size() {
        return this.#size;
    }
    get get_size() {
        return this.get_offset + this.get_field_size;
    }
    get get_nodeid() {
        return this.#nodeid;
    }
    get render() {
        return '';
    }
}
class CdiField extends CdiBaseElement {
    #min_value;
    #max_value;
    #default_value;
    #mapped_values;
    constructor(node, origin, space, nodeid) {
        super(node, origin, space, nodeid);
        this.#min_value = 0;
        this.#max_value = 0;
        this.#default_value = 0;
        this.#mapped_values = [];
        $(node.children).each((x, child) => {
            if (child.nodeName === 'map') {
                $(child.children).each((x, entry) => {
                    this.#mapped_values.push({
                        key: $(entry).find('property').text(),
                        value: $(entry).find('value').text()
                    });
                });
            } else if (child.nodeName === 'min') {
                this.#min_value = parseInt($(child).text()) || 0;
            } else if (child.nodeName === 'max') {
                this.#max_value = parseInt($(child).text()) || 0;
            } else if (child.nodeName === 'default') {
                this.#default_value = parseInt($(child).text()) || 0;
            }
        });
        if (this.#mapped_values.length) {
            console.groupCollapsed(
                String.format('MappedField (name:"{0}", descr:"{1}", offs:{2}, size:{3}, type:"{4}", min:{5}, max:{6}, default:{7}, alias:{8})',
                    this.get_name, this.get_description, this.get_address, this.get_field_size,
                    this.get_type, this.#min_value, this.#max_value, this.#default_value, this.get_alias));
            this.#mapped_values.forEach(entry => {
                console.log(String.format('key:{0}, value:{1}', entry.key, entry.value));
            });
            console.groupEnd();
        } else if (this.get_type === 'int') {
            console.log(
                String.format('Field (name:"{0}", descr:"{1}", offs:{2}, size:{3}, type:"{4}", min:{5}, max:{6}, default:{7}, alias:{8})',
                    this.get_name, this.get_description, this.get_address, this.get_field_size,
                    this.get_type, this.#min_value, this.#max_value, this.#default_value, this.get_alias));
        } else {
            console.log(
                String.format('Field (name:"{0}", descr:"{1}", offs:{2}, size:{3}, type:"{4}", alias:{5})',
                    this.get_name, this.get_description, this.get_address, this.get_field_size, this.get_type, this.get_alias));
        }
        return this;
    }
    get render() {
        var content = '<div class="columns"><div class="column">';
        if (this.has_name) {
            content += String.format('<label class="form-label" for="{0}">{1}:</label>', this.get_alias, this.get_name);
        }
        content += '</div>';
        if (this.#mapped_values.length) {
            content +=
                String.format('<div class="column column-3"><select class="form-select text-dark" id="{0}" data-offset="{1}" data-type="{2}" data-size="{3}" data-node="{4}" data-space="{5}" oninput="enable_button(\'save-{0}\')">',
                    this.get_alias, this.get_address, this.get_type, this.get_field_size, this.get_nodeid, this.get_space);
            this.#mapped_values.forEach(entry => {
                content += String.format('<option value="{0}">{1}</option>', entry.key, entry.value);
            });
            content += '</select></div>';
        } else {
            content +=
                String.format('<div class="column column-3"><input class="form-input" type="text" id="{0}" data-offset="{1}" data-type="{2}" data-size="{3}" data-node="{4}" data-space="{5}" oninput="enable_button(\'save-{0}\')"></div>',
                    this.get_alias, this.get_address, this.get_type, this.get_field_size, this.get_nodeid, this.get_space);
        }
        content += '<div class="column"><div class="btn-group">';
        content +=
            String.format('<button class="btn btn-primary btn-sm" id="btn-save-{0}" disabled onclick="save_cdi_field(this);">Save</button>',
                this.get_alias);
        content +=
            String.format('<button class="btn btn-primary btn-sm loading" id="btn-refresh-{0}" onclick="refresh_cdi_field(this);">Refresh</button>',
                this.get_alias);
        content += '</div></div></div>';
        if (this.has_description) {
            content += String.format('<div class="columns"><div class="column"><small>{0}</small></div></div>', this.get_description);
        }
        cdi_fields.push({
            key: this.get_alias,
            msg: JSON.stringify({
                req: 'cdi',
                ofs: this.get_address,
                sz: this.get_field_size,
                type: this.get_type,
                tgt: this.get_alias,
                node: this.get_nodeid,
                spc: this.get_space,
                id: get_ws_msg_id()
            })
        });
        return content;
    }
}
class EmptyCdiGroup extends CdiBaseElement {
    constructor(node, origin, space, nodeid) {
        super(node, origin, space, nodeid);
        console.log(String.format('EmptyGroup (offs:{0}, size:{1})', this.get_address, this.get_size));
    }
}
class ReplicatedCdiGroup extends CdiBaseElement {
    #items;
    constructor(node, origin, spaceid, nodeid) {
        super(node, origin, spaceid, nodeid);
        this.#items = [];
        var rep_count = parseInt($(node).attr('replication'));
        var offset = this.get_address;
        var rep_name = this.get_name;
        $(node.children).each((x, child) => {
            if (child.nodeName === 'repname') {
                rep_name = $(child).text();
            }
        });
        console.groupCollapsed(
            String.format('ReplicatedGroup (name:"{0}", repname:"{1}", descr:"{2}", addr:{3}, count:{4}, alias:{5})',
                this.get_name, rep_name, this.get_description, this.get_address, rep_count, this.get_alias));
        Array.from({ length: rep_count }, (x, index) => {
            var name = String.format('{0} {1}', rep_name, index + 1);
            console.groupCollapsed(name);
            var members = [];
            $(node.children).each((x, child) => {
                if (child.nodeName === 'group') {
                    if ((parseInt($(child).attr('replication')) || 1) > 1) {
                        var group = new ReplicatedCdiGroup(child, offset, this.get_space, this.get_nodeid);
                        members.push(group);
                        offset += group.get_size;
                    } else if (child.children.length >= 1) {
                        var group = new CdiGroup(child, offset, this.get_space, this.get_nodeid);
                        members.push(group);
                        offset += group.get_size;
                    } else {
                        var group = new EmptyCdiGroup(child, offset, this.get_space, this.get_nodeid);
                        members.push(group);
                        offset += group.get_size;
                    }
                } else if (child.nodeName === 'int' ||
                           child.nodeName === 'string' ||
                           child.nodeName === 'eventid') {
                    var field = new CdiField(child, offset, this.get_space, this.get_nodeid);
                    members.push(field);
                    offset += field.get_size;
                }
            });
            this.#items.push({
                name: name,
                items: members
            });
            console.groupEnd();
        });
        console.groupEnd();
        console.debug(String.format('ReplicatedGroup("{0}") size:{1}', this.get_name, this.get_size));
    }
    get render() {
        var content = '<fieldset>';
        if (this.has_name) {
            content += String.format('<legend>{0}:</legend>', this.get_name);
        }
        if (this.has_description) {
            content += String.format('<div class="columns"><div class="column">{0}</div></div>', this.get_description);
        }
        content += '<ul class="tab tab-block">';
        var rep_fields = '';
        var tab_class = 'active';
        var tab_state = 'block';
        $(this.#items).each((idx, item) => {
            content +=
                String.format('<li class="tab-item"><a href="#" id="{0}-{1}" class="{3}" onclick="show_group_tab(this)">{2}</a></li>',
                    this.get_alias, idx, item.name, tab_class);
            rep_fields +=
                String.format('<div class="container" id="rep-{0}-{1}" style="display:{2};">',
                    this.get_alias, idx, tab_state);
            item.items.forEach(field => {
                rep_fields += field.render;
            });
            rep_fields += '</div>';
            tab_class = 'inactive';
            tab_state = 'none';
        });
        content += '</ul>';
        content += rep_fields;
        content += '</fieldset>';
        return content;
    }
    get get_size() {
        var size = this.get_offset;
        this.#items.forEach(item => {
            item.items.forEach(field => {
                size += field.get_size;
            });
        });
        return size;
    }
}
class CdiGroup extends CdiBaseElement {
    #items;
    constructor(node, origin, spaceid, nodeid) {
        super(node, origin, spaceid, nodeid);
        this.#items = [];
        console.groupCollapsed(
            String.format('Group (name:"{0}", descr:"{1}", addr:{2}, alias:{3})',
                this.get_name, this.get_description, this.get_address, this.get_alias));
        var offset = this.get_address;
        $(node.children).each((x, child) => {
            if (child.nodeName === 'group') {
                if ((parseInt($(child).attr('replication')) || 1) > 1) {
                    var group = new ReplicatedCdiGroup(child, offset, this.get_space, this.get_nodeid);
                    this.#items.push(group);
                    offset += group.get_size;
                } else if (child.children.length >= 1) {
                    var group = new CdiGroup(child, offset, this.get_space, this.get_nodeid);
                    this.#items.push(group);
                    offset += group.get_size;
                } else {
                    var group = new EmptyCdiGroup(child, offset, this.get_space, this.get_nodeid);
                    this.#items.push(group);
                    offset += group.get_size;
                }

            } else if (child.nodeName === 'int' ||
                       child.nodeName === 'string' ||
                       child.nodeName === 'eventid') {
                var field = new CdiField(child, offset, this.get_space, this.get_nodeid);
                this.#items.push(field);
                offset += field.get_size;
            }
        });
        console.groupEnd();
        console.debug(String.format('Group("{0}") size:{1}', this.get_name, this.get_size));
        return this;
    }
    get render() {
        var content = '<fieldset>';
        if (this.has_name) {
            content += String.format('<legend>{0}:</legend>', this.get_name);
        }
        if (this.has_description) {
            content += String.format('<div class="columns"><div class="column">{0}</div></div>', this.get_description);
        }
        this.#items.forEach(item => {
            content += item.render;
        });
        content += '</fieldset>';
        return content;
    }
    get get_size() {
        var size = this.get_offset;
        this.#items.forEach(item => {
            size += item.get_size;
        });
        return size;
    }
}
class CdiSegment extends CdiBaseElement {
    #items;
    constructor(node, nodeid) {
        super(node, parseInt($(node).attr('origin')) || 0, parseInt($(node).attr('space')), nodeid);
        this.#items = [];
        var offset = this.get_address;
        console.groupCollapsed(String.format('Segment (name:"{0}", descr:"{1}", space:{2}, origin:{3})',
            this.get_name, this.get_description, this.get_space, this.get_address));
        $(node.children).each((x, child) => {
            if (child.nodeName === 'group') {
                if ((parseInt($(child).attr('replication')) || 1) > 1) {
                    var group = new ReplicatedCdiGroup(child, offset, this.get_space, this.get_nodeid);
                    this.#items.push(group);
                    offset += group.get_size;
                } else if (child.children.length >= 1) {
                    var group = new CdiGroup(child, offset, this.get_space, this.get_nodeid);
                    this.#items.push(group);
                    offset += group.get_size;
                } else {
                    var group = new EmptyCdiGroup(child, offset, this.get_space, this.get_nodeid);
                    this.#items.push(group);
                    offset += group.get_size;
                }
            } else if (child.nodeName === 'int' ||
                       child.nodeName === 'string' ||
                       child.nodeName === 'eventid') {
                var field = new CdiField(child, offset, this.get_space, this.get_nodeid);
                this.#items.push(field);
                offset += field.get_size;
            }
        });
        console.groupEnd();
    }
    render(target) {
        var content = '';
        this.#items.forEach(item => {
            content += item.render;
        });
        $(target).append(content);
    }
    is_space(space) {
        return this.get_space === space;
    }
}
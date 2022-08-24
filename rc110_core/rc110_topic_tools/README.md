# RC 1/10 Topic Tools

Set of nodes to supplement standard topic_tools.

## Multi Demux
### Subscribed Topics
```
~/input_ns [std_msgs::String]
    input namespace (empty by default)
    
~/output_ns [std_msgs::String]
    output namespace (empty by default)
    
{/input_ns/}<topicN> [<any>::<any>]
    input topics
```

### Published Topics
```
{/output_ns/}<topicN> [<any>::<any>]
    relayed topics
```

### Parameters
```    
~/topics (string[], default: [])
    input topic names
```

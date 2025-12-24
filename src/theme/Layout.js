import React from 'react';
import Layout from '@theme-original/Layout';
import GlobalChatWidget from '../components/GlobalChatWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
      </Layout>
      <GlobalChatWidget />
    </>
  );
}
import React from 'react';
import ChatWidget from '../components/chat_widget';
import Layout from '@theme/Layout';

export default function Root({children}) {
  return (
    <Layout>
      {children}
      <ChatWidget />
    </Layout>
  );
}